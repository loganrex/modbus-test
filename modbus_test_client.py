#!/usr/bin/env python3
# Works with: pymodbus==2.4.*, pyserial
# TCP / RTU-over-TCP / Serial RTU client with:
# - Chaos on outbound (delay/split/mangle)
# - Burst sending and Speed test
# - Sequence verification (for read responses)
# - NEW: --seed-seq VALUE (writes holding register 'seed-seq-addr', default 0) before tests

import argparse
import math
import random
import socket
import struct
import time

import serial

from pymodbus.factory import ClientDecoder
from pymodbus.framer.socket_framer import ModbusSocketFramer
from pymodbus.framer.rtu_framer import ModbusRtuFramer

from pymodbus.bit_read_message import ReadCoilsRequest, ReadDiscreteInputsRequest
from pymodbus.register_read_message import ReadHoldingRegistersRequest, ReadInputRegistersRequest
from pymodbus.bit_write_message import WriteSingleCoilRequest
from pymodbus.register_write_message import WriteSingleRegisterRequest

# -------------------- Chaos config --------------------
class Chaos:
    enabled = False
    rate_percent = 100
    delay_ms = 0
    split_at = 0
    split_gap_ms = 0
    mangle_prob = 0.0
    mangle_crc = False

CHAOS = Chaos()

# -------------------- Verify seq config --------------------
class VerifySeq:
    enabled = False
    expected = 0
    increment = 1
    wrap = 65536

VERIFY = VerifySeq()

# -------------------- Helpers --------------------
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def _flip_random_bit(b: int) -> int:
    return b ^ (1 << random.randrange(8))

def mangle_frame(frame: bytes, is_rtu_like: bool) -> bytes:
    if CHAOS.mangle_prob <= 0.0 or random.random() > CHAOS.mangle_prob or len(frame) < 5:
        return frame
    if is_rtu_like:
        if CHAOS.mangle_crc:
            lo, hi = frame[-2], frame[-1]
            if random.random() < 0.5: lo = _flip_random_bit(lo)
            else: hi = _flip_random_bit(hi)
            return frame[:-2] + bytes([lo, hi])
        payload = bytearray(frame[1:-2])
        if payload:
            i = random.randrange(len(payload))
            payload[i] = _flip_random_bit(payload[i])
        raw = bytes([frame[0]]) + bytes(payload)
        return raw + struct.pack(\"<H\", crc16_modbus(raw))
    adu = bytearray(frame)
    if len(adu) >= 8:
        idx = random.randrange(7, len(adu))
        adu[idx] = _flip_random_bit(adu[idx])
    return bytes(adu)

def maybe_chaos_bytes(adu: bytes, is_rtu_like: bool) -> bytes:
    use = CHAOS.enabled and CHAOS.rate_percent > 0 and random.randrange(100) < CHAOS.rate_percent
    out = adu
    if use:
        out = mangle_frame(out, is_rtu_like)
        if CHAOS.delay_ms > 0:
            time.sleep(CHAOS.delay_ms / 1000.0)
    return out

def split_send(write_fn, adu: bytes):
    if CHAOS.enabled and CHAOS.split_at > 0 and CHAOS.split_at < len(adu):
        first, rest = adu[:CHAOS.split_at], adu[CHAOS.split_at:]
        write_fn(first)
        if CHAOS.split_gap_ms > 0:
            time.sleep(CHAOS.split_gap_ms / 1000.0)
        write_fn(rest)
    else:
        write_fn(adu)

# -------------------- Build requests --------------------
def build_request(op: str, addr: int, count: int, value: int, unit: int):
    op = (op or \"\").lower()
    if op in (\"read-coils\", \"rc\"):
        req = ReadCoilsRequest(address=addr, count=count, unit=unit)
    elif op in (\"read-discretes\", \"rdi\", \"read-discrete-inputs\"):
        req = ReadDiscreteInputsRequest(address=addr, count=count, unit=unit)
    elif op in (\"read-holding\", \"rhr\", \"read-holding-registers\"):
        req = ReadHoldingRegistersRequest(address=addr, count=count, unit=unit)
    elif op in (\"read-input\", \"rir\", \"read-input-registers\"):
        req = ReadInputRegistersRequest(address=addr, count=count, unit=unit)
    elif op in (\"write-coil\", \"wsc\", \"write-single-coil\"):
        req = WriteSingleCoilRequest(address=addr, value=bool(value), unit=unit)
    elif op in (\"write-register\", \"wsr\", \"write-single-register\"):
        req = WriteSingleRegisterRequest(address=addr, value=value & 0xFFFF, unit=unit)
    else:
        raise ValueError(f\"Unknown op: {op}\")
    setattr(req, \"unit_id\", unit)
    setattr(req, \"unit\", unit)
    return req

# -------------------- Seq extraction & verification --------------------
def extract_seq_from_response(resp):
    try:
        fc = int(getattr(resp, \"function_code\"))
    except Exception:
        return (False, 0, 0, None)
    if fc & 0x80:
        return (False, 0, 0, fc)
    if fc in (0x03, 0x04):
        regs = getattr(resp, \"registers\", None)
        if isinstance(regs, list) and len(regs) >= 1:
            return (True, int(regs[0]) & 0xFFFF, 16, fc)
        return (False, 0, 0, fc)
    if fc in (0x01, 0x02):
        bits = getattr(resp, \"bits\", None)
        if isinstance(bits, list) and len(bits) >= 1:
            n = min(len(bits), 16)
            val = 0
            for i in range(n):
                if bits[i]:
                    val |= (1 << i)
            return (True, val, n, fc)
        return (False, 0, 0, fc)
    return (False, 0, 0, fc)

def verify_seq(resp):
    if not VERIFY.enabled:
        return (True, None, None, 0, \"verification disabled\")
    present, got, present_bits, fc = extract_seq_from_response(resp)
    if not present:
        return (False, None, None, 0, f\"no sequence field present (fc=0x{(fc or 0):02X})\")
    mask = (1 << present_bits) - 1 if present_bits > 0 else 0xFFFF
    want = VERIFY.expected & mask
    ok = ((got & mask) == want)
    msg = f\"seq {'OK' if ok else 'FAIL'}: got=0x{got:04X}, want=0x{VERIFY.expected & 0xFFFF:04X}, bits={present_bits}, fc=0x{fc:02X}\"
    VERIFY.expected = (VERIFY.expected + VERIFY.increment) % max(1, VERIFY.wrap)
    return (ok, got, want, present_bits, msg)

# -------------------- Client core --------------------
class ChaosClient:
    def __init__(self, mode: str, host: str, port: int,
                 serial_port: str, baudrate: int, bytesize: int, parity: str, stopbits: int,
                 timeout: float, rtu_over_tcp: bool):
        self.mode = mode
        self.timeout = timeout
        self.sock = None
        self.ser = None
        self.is_rtu_like = False
        self.decoder = ClientDecoder()

        if mode == \"tcp\":
            self.framer = ModbusSocketFramer(self.decoder)
            self.sock = socket.create_connection((host, port), timeout=timeout)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.is_rtu_like = False
        elif mode == \"rtu_tcp\":
            self.framer = ModbusRtuFramer(self.decoder)
            self.sock = socket.create_connection((host, port), timeout=timeout)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.is_rtu_like = True
        elif mode == \"serial\":
            self.framer = ModbusRtuFramer(self.decoder)
            self.ser = serial.Serial(
                port=serial_port, baudrate=baudrate, bytesize=bytesize,
                parity=parity, stopbits=stopbits, timeout=timeout, write_timeout=timeout
            )
            self.is_rtu_like = True
        else:
            raise ValueError(\"mode must be one of: tcp, rtu_tcp, serial\")

    def close(self):
        try:
            if self.sock: self.sock.close()
        except Exception: pass
        try:
            if self.ser: self.ser.close()
        except Exception: pass

    def _write(self, b: bytes):
        if self.sock: self.sock.sendall(b)
        else: self.ser.write(b); self.ser.flush()

    def _read_some(self) -> bytes:
        if self.sock:
            try:
                return self.sock.recv(4096)
            except socket.timeout:
                return b\"\"
        else:
            n = self.ser.in_waiting if hasattr(self.ser, \"in_waiting\") else 0
            return self.ser.read(n or 1)

    def transact(self, req, unit: int, verbose: bool = True):
        t0 = time.perf_counter()
        adu = self.framer.buildPacket(req)
        if verbose:
            print(f\"[out] {len(adu)}B {adu.hex()}\")
        adu = maybe_chaos_bytes(adu, self.is_rtu_like)
        split_send(self._write, adu)

        start = time.time()
        result = {\"resp\": None, \"raw\": b\"\", \"timed_out\": False, \"exception\": False, \"latency_ms\": None}
        def on_resp(resp): result[\"resp\"] = resp

        while (time.time() - start) < self.timeout:
            buf = self._read_some()
            if not buf: continue
            result[\"raw\"] += buf
            self.framer.processIncomingPacket(buf, on_resp, unit=unit)
            if result[\"resp\"] is not None: break

        result[\"latency_ms\"] = (time.perf_counter() - t0) * 1000.0
        if result[\"resp\"] is None:
            result[\"timed_out\"] = True
            if verbose:
                print(f\"[timeout] {self.timeout:.3f}s; received {len(result['raw'])}B: {result['raw'].hex()}\")
        else:
            fc = int(getattr(result[\"resp\"], \"function_code\", 0))
            result[\"exception\"] = bool(fc & 0x80)
            if verbose:
                print(f\"[in ] {result['resp']}  ({result['latency_ms']:.2f} ms)\")
        return result

# -------------------- Percentile helper --------------------
def percentile(values, p):
    if not values: return float(\"nan\")
    s = sorted(values)
    if len(s) == 1: return s[0]
    k = (p / 100.0) * (len(s) - 1)
    f = int(k); c = min(len(s) - 1, f + 1)
    if f == c: return s[f]
    return s[f] + (s[c] - s[f]) * (k - f)

# -------------------- CLI --------------------
def main():
    ap = argparse.ArgumentParser(description=\"Chaos Modbus client (pymodbus 2.4 sync) — TCP / RTU-over-TCP / Serial RTU\")
    # Transport
    ap.add_argument(\"--mode\", choices=[\"tcp\", \"rtu_tcp\", \"serial\"], required=True)
    ap.add_argument(\"--host\", default=\"127.0.0.1\")
    ap.add_argument(\"--port\", type=int, default=502)
    ap.add_argument(\"--serial-port\")
    ap.add_argument(\"--baudrate\", type=int, default=9600)
    ap.add_argument(\"--bytesize\", type=int, choices=[7, 8], default=8)
    ap.add_argument(\"--parity\", choices=[\"N\", \"E\", \"O\"], default=\"N\")
    ap.add_argument(\"--stopbits\", type=int, choices=[1, 2], default=1)
    ap.add_argument(\"--timeout\", type=float, default=2.0)
    ap.add_argument(\"--unit\", type=int, default=1)

    # Op
    ap.add_argument(\"--op\", required=True,
                    help=\"read-coils|read-discretes|read-holding|read-input|write-coil|write-register "
                         \"(aliases: rc,rdi,rhr,rir,wsc,wsr)\")
    ap.add_argument(\"--addr\", type=int, default=0)
    ap.add_argument(\"--count\", type=int, default=1)
    ap.add_argument(\"--value\", type=int, default=1)

    # Chaos
    ap.add_argument(\"--enable-chaos\", action=\"store_true\")
    ap.add_argument(\"--chaos-rate\", type=int, default=100)
    ap.add_argument(\"--delay\", type=int, default=0)
    ap.add_argument(\"--split-at\", type=int, default=0)
    ap.add_argument(\"--split-gap\", type=int, default=0)
    ap.add_argument(\"--mangle-prob\", type=float, default=0.0)
    ap.add_argument(\"--mangle-crc\", action=\"store_true\")

    # Burst / speed test
    ap.add_argument(\"--burst\", type=int, default=1)
    ap.add_argument(\"--req-interval-ms\", type=int, default=0)
    ap.add_argument(\"--speed\", type=int, default=0)
    ap.add_argument(\"--quiet\", action=\"store_true\")
    ap.add_argument(\"--progress-every\", type=int, default=0)

    # Verify seq
    ap.add_argument(\"--verify-seq\", action=\"store_true\")
    ap.add_argument(\"--verify-seq-start\", type=int, default=0)
    ap.add_argument(\"--verify-seq-increment\", type=int, default=1)
    ap.add_argument(\"--verify-seq-wrap\", type=int, default=65536)

    # NEW: seed sequence (write HR[0] before tests)
    ap.add_argument(\"--seed-seq\", type=int, help=\"If set, write this value to holding register at --seed-seq-addr before running\")
    ap.add_argument(\"--seed-seq-addr\", type=int, default=0, help=\"Holding register address to write the seed (default 0)\")

    args = ap.parse_args()

    # Chaos cfg
    CHAOS.enabled = args.enable_chaos
    CHAOS.rate_percent = max(0, min(100, args.chaos_rate))
    CHAOS.delay_ms = max(0, args.delay)
    CHAOS.split_at = max(0, args.split_at)
    CHAOS.split_gap_ms = max(0, args.split_gap)
    CHAOS.mangle_prob = max(0.0, min(1.0, args.mangle_prob))
    CHAOS.mangle_crc = bool(args.mangle_crc)

    # Verify cfg
    VERIFY.enabled = args.verify_seq
    VERIFY.expected = args.verify_seq_start & 0xFFFF
    VERIFY.increment = args.verify_seq_increment
    VERIFY.wrap = max(1, args.verify_seq_wrap)

    # Validate serial params if needed
    if args.mode == \"serial\" and not args.serial_port:
        raise SystemExit(\"--serial-port is required when --mode serial\")

    client = ChaosClient(
        mode=args.mode,
        host=args.host, port=args.port,
        serial_port=args.serial_port, baudrate=args.baudrate,
        bytesize=args.bytesize, parity=args.parity, stopbits=args.stopbits,
        timeout=args.timeout, rtu_over_tcp=(args.mode == \"rtu_tcp\"),
    )

    # OPTIONALLY seed the server counter via Modbus write
    if args.seed_seq is not None:
        seed_req = WriteSingleRegisterRequest(address=args.seed_seq_addr, value=args.seed_seq & 0xFFFF, unit=args.unit)
        setattr(seed_req, \"unit_id\", args.unit)
        print(f\"[seed] Writing 0x{args.seed_seq & 0xFFFF:04X} to HR[{args.seed_seq_addr}] (unit {args.unit})\")
        seed_res = client.transact(seed_req, unit=args.unit, verbose=not args.quiet)
        if seed_res[\"resp\"] is None:
            print(\"[seed] WARN: seeding timed out\")
        elif seed_res[\"exception\"]:
            print(\"[seed] WARN: exception on seed write\")
        else:
            # Optional: read-back verify
            rb = client.transact(ReadHoldingRegistersRequest(address=args.seed_seq_addr, count=1, unit=args.unit),
                                 unit=args.unit, verbose=not args.quiet)
            if rb[\"resp\"] is not None and not rb[\"exception\"]:
                regs = getattr(rb[\"resp\"], \"registers\", [])
                if regs:
                    print(f\"[seed] Read-back HR[{args.seed_seq_addr}] = 0x{regs[0] & 0xFFFF:04X}\")

    # Build main request once
    req = build_request(args.op, args.addr, args.count, args.value, args.unit)

    # SPEED TEST path
    if args.speed > 0:
        N = args.speed
        total = responses = timeouts = exceptions = 0
        verified = passed = 0
        latencies = []
        t_start = time.perf_counter()
        try:
            for i in range(N):
                verbose = not args.quiet
                if args.progress_every > 0:
                    verbose = False
                    if (i % args.progress_every) == 0:
                        print(f\"[progress] {i}/{N} sent...\")
                res = client.transact(req, unit=args.unit, verbose=verbose)
                total += 1
                if res[\"latency_ms\"] is not None: latencies.append(res[\"latency_ms\"])
                if res[\"resp\"] is None: timeouts += 1
                else:
                    responses += 1
                    if res[\"exception\"]: exceptions += 1
                    if VERIFY.enabled and args.op.lower() in (
                        \"read-coils\",\"rc\",\"read-discretes\",\"rdi\",\"read-discrete-inputs\",
                        \"read-holding\",\"rhr\",\"read-holding-registers\",
                        \"read-input\",\"rir\",\"read-input-registers\"
                    ):
                        ok, _, _, _, _ = verify_seq(res[\"resp\"])
                        verified += 1
                        if ok: passed += 1
        finally:
            client.close()

        wall_ms = (time.perf_counter() - t_start) * 1000.0
        tput = (total / (wall_ms / 1000.0)) if wall_ms > 0 else float(\"nan\")
        lat_min = min(latencies) if latencies else float(\"nan\")
        lat_max = max(latencies) if latencies else float(\"nan\")
        lat_avg = (sum(latencies) / len(latencies)) if latencies else float(\"nan\")
        lat_p50 = percentile(latencies, 50) if latencies else float(\"nan\")
        lat_p90 = percentile(latencies, 90) if latencies else float(\"nan\")
        lat_p99 = percentile(latencies, 99) if latencies else float(\"nan\")

        print(\"\\n=== Speed Test Summary ===\")
        print(f\"mode={args.mode} target={args.host}:{args.port} unit={args.unit} \"
              f\"op={args.op} addr={args.addr} count={args.count} timeout={args.timeout}s\")
        print(f\"requests: {total}  responses: {responses}  timeouts: {timeouts}  exceptions: {exceptions}\")
        if VERIFY.enabled and verified > 0:
            print(f\"verify:   checked={verified}  passed={passed}  failed={verified - passed}\")
        print(f\"time:     {wall_ms:.2f} ms  throughput: {tput:.2f} req/s\")
        print(\"latency (ms): \"
              f\"min={lat_min:.2f} avg={lat_avg:.2f} p50={lat_p50:.2f} p90={lat_p90:.2f} p99={lat_p99:.2f} max={lat_max:.2f}\")
        return

    # Non-speed-test (burst)
    total = checked = passed = 0
    try:
        for i in range(max(1, args.burst)):
            print(f\"\\n=== Request {i+1}/{args.burst} ===\")
            res = client.transact(req, unit=args.unit, verbose=not args.quiet)
            total += 1
            if VERIFY.enabled and res[\"resp\"] is not None and args.op.lower() in (
                \"read-coils\",\"rc\",\"read-discretes\",\"rdi\",\"read-discrete-inputs\",
                \"read-holding\",\"rhr\",\"read-holding-registers\",
                \"read-input\",\"rir\",\"read-input-registers\"
            ):
                ok, _, _, _, msg = verify_seq(res[\"resp\"])
                checked += 1
                if ok: passed += 1
                print(f\"[verify] {msg}\")
            if i + 1 < args.burst and args.req_interval_ms > 0:
                time.sleep(args.req_interval_ms / 1000.0)
    finally:
        client.close()

    if VERIFY.enabled and checked > 0:
        print(f\"\\n=== Verification Summary ===\")
        print(f\"requests: {total}, checked: {checked}, passed: {passed}, failed: {checked - passed}\")

if __name__ == \"__main__\":
    main()
