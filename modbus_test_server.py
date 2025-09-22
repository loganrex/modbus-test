#!/usr/bin/env python3
# Requires: pymodbus==2.4.*, pyserial
import argparse
import random
import struct
import time
from typing import Callable

import serial

from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.factory import ServerDecoder
from pymodbus.framer.rtu_framer import ModbusRtuFramer
from pymodbus.pdu import ModbusRequest, ExceptionResponse

# -------------------- Chaos controls --------------------
class Chaos:
    enabled: bool = False
    rate_percent: int = 100
    delay_ms: int = 0
    split_at: int = 0
    split_gap_ms: int = 0
    mangle_prob: float = 0.0
    mangle_crc: bool = False

CHAOS = Chaos()

# -------------------- Sequence settings --------------------
class SeqCfg:
    enable: bool = True       # enable injecting from HR[0]
    increment: int = 1        # how much to add after each injected response
    wrap: int = 65536         # modulo for the counter

SEQ = SeqCfg()

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

def _flip_random_bit(byte_val: int) -> int:
    return byte_val ^ (1 << random.randrange(8))

def mangle_rtu_frame(frame: bytes) -> bytes:
    """Mangle an RTU frame [addr][PDU...][CRClo][CRChi]."""
    if CHAOS.mangle_prob <= 0.0 or random.random() > CHAOS.mangle_prob or len(frame) < 5:
        return frame

    if CHAOS.mangle_crc:
        lo, hi = frame[-2], frame[-1]
        if random.random() < 0.5:
            lo = _flip_random_bit(lo)
        else:
            hi = _flip_random_bit(hi)
        return frame[:-2] + bytes([lo, hi])

    # Flip a bit in the payload AFTER CRC recomputed - we purposely leave CRC unchanged
    payload_len = len(frame) - 3  # addr + pdu excluding crc -> payload region starts at index 1 with length payload_len
    if payload_len <= 0:
        return frame
    # pick an index into payload (1..1+payload_len-1) -> absolute index in frame
    i = 1 + random.randrange(payload_len)
    b = frame[i]
    b2 = _flip_random_bit_in_byte(b)
    out = bytearray(frame)
    out[i] = b2
    return bytes(out)


def inject_seq_from_context_and_increment(frame: bytes, context: ModbusServerContext,
                                          unit: int, increment: int, wrap: int, request_obj: ModbusRequest=None) -> bytes:
    """
    Read seq from HR[0], inject into first register (FC 03/04) or first 16 bits (FC 01/02),
    then increment and save back to HR[0]. CRC is recomputed. If anything fails, return frame.
    """
    if (not SEQ.enable) or len(frame) < 5:
        return frame

    try:
        addr = frame[0]
        pdu = bytearray(frame[1:-2])
        if not pdu:
            return frame

        fc = pdu[0]
        if fc & 0x80:
            return frame  # exception responses untouched

        if fc not in (0x01, 0x02, 0x03, 0x04):
            return frame  # only read responses

        if len(pdu) < 2:
            return frame
        byte_count = pdu[1]
        if len(pdu) < 2 + byte_count or byte_count == 0:
            return frame

        # read current seq from HR[0]
        try:
            cur = int(context[unit].getValues(3, 0, count=1)[0]) & 0xFFFF
        except Exception:
            cur = 0  # fallback if HR[0] not present

        lo = cur & 0xFF
        hi = (cur >> 8) & 0xFF
        changed = False

        if fc in (0x03, 0x04):
            # For register reads we want to inject two registers (4 data bytes) at the start:
            # data layout becomes: [seq_hi][seq_lo][addr_hi][addr_lo][orig_reg_hi_0][orig_reg_lo_0]...
            # This requires that the original byte_count >= 4 (i.e., at least two registers were to be returned).
            if byte_count >= 4:
                # shift original data right by 4 bytes to make space for seq+addr if necessary.
                # pdu currently = [fc, byte_count, data0, data1, ...]
                #orig_data = pdu[2:2+byte_count]
                # build new_data = seq_hi, seq_lo, addr_hi, addr_lo, orig_data...
                if request_obj is not None and hasattr(request_obj, "address"):
                    req_addr = int(getattr(request_obj, "address") or 0) & 0xFFFF
                else:
                    req_addr = 0
                addr_hi = (req_addr >> 8) & 0xFF
                addr_lo = req_addr & 0xFF
                pdu[2] = hi
                pdu[3] = lo
                pdu[4] = addr_hi
                pdu[5] = addr_lo
                changed = True
            else:
                # Not enough returned bytes to place seq+addr; fall back to injecting seq into first register only if possible
                if byte_count >= 2:
                    pdu[2] = hi
                    pdu[3] = lo
                    changed = True

        else:
            # Bits: put into first 16 bits (LSB-first). If only 1 byte, write low 8 bits.
            if byte_count >= 1:
                pdu[2] = lo
                changed = True
            if byte_count >= 2:
                pdu[3] = hi

        if not changed:
            return frame

        # rebuild raw frame (addr + pdu) and compute CRC
        raw = bytes([addr]) + bytes(pdu)
        crc = crc16_modbus(raw)
        crc_bytes = struct.pack("<H", crc)
        full = raw + crc_bytes

        # increment and save back to HR[0]
        nxt = (cur + (increment % wrap)) % wrap
        try:
            context[unit].setValues(3, 0, [nxt & 0xFFFF])
        except Exception as e:
            print("Warning: failed to increment HR[0] on seq wrap ",e, flush=True)

        if nxt < cur:
            try:
                context[unit].setValues(3, 1, [int(context[unit].getValues(3, 1, count=1)[0]+1) & 0xFFFF])
            except Exception as e:
                print("Warning: failed to increment HR[1] on seq wrap ", e, flush=True)


        return full
    except Exception:
        return frame

def chaos_send(ser: serial.Serial, frame: bytes, request_obj: ModbusRequest=None):
    """
    1) Inject sequence from HR[0] (if enabled) and save back incremented value.
    2) Optionally apply chaos (selected by CHAOS.rate_percent).
    3) Send (with optional split & delay).
    """
    out = inject_seq_from_context_and_increment(frame, SERVER_CONTEXT, CURRENT_UNIT,
                                               SEQ.increment, max(1, SEQ.wrap), request_obj)
    print(f"[chaos] send {len(out)} bytes: {out.hex()}", flush=True)
    use_chaos = CHAOS.enabled and CHAOS.rate_percent > 0 and random.randrange(100) < CHAOS.rate_percent
    if use_chaos:
        out = mangle_rtu_frame(out)
        if CHAOS.delay_ms > 0:
            time.sleep(CHAOS.delay_ms / 1000.0)

        if CHAOS.split_at > 0 and CHAOS.split_at < len(out):
            first = out[:CHAOS.split_at]
            rest = out[CHAOS.split_at:]
            ser.write(first); ser.flush()
            if CHAOS.split_gap_ms > 0:
                time.sleep(CHAOS.split_gap_ms / 1000.0)
            ser.write(rest); ser.flush()
            return

    ser.write(out); ser.flush()

# -------------------- Server core (sync, manual I/O) --------------------
# We'll keep minimal globals so chaos_send() can access context/unit cleanly.
SERVER_CONTEXT = None
CURRENT_UNIT = 1

class RtuChaosServer(object):
    def __init__(self, port: str, baudrate: int, bytesize: int, parity: str, stopbits: int,
                 unit: int, context: ModbusServerContext, timeout: float = 1.0):
        global SERVER_CONTEXT, CURRENT_UNIT
        self.unit = unit
        self.context = context
        SERVER_CONTEXT = context
        CURRENT_UNIT = unit

        self.ser = serial.Serial(
            port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,
            stopbits=stopbits, timeout=timeout, write_timeout=timeout
        )
        self.decoder = ServerDecoder()
        self.framer = ModbusRtuFramer(self.decoder)

    def _handle_request(self, request: ModbusRequest):
        try:
            request.unit_id = getattr(request, "unit_id", self.unit)
            if request.unit_id != self.unit:
                print(f"[req] ignoring request for unit {request.unit_id}", flush=True)
                return
            response = request.execute(self.context[self.unit])
            #response.unit_id = getattr(response, "unit_id", request.unit_id)
            response.unit_id = self.unit
            print(f"[req] {request.unit_id} {request} -> {response.unit_id} {response}", flush=True)
        except Exception:
            response = ExceptionResponse(0x00)
            response.unit_id = self.unit

        packet = self.framer.buildPacket(response)
        if packet:
            chaos_send(self.ser, packet, request_obj=request)

    def serve_forever(self):
        print("[rtu] listening...")
        try:
            while True:
                n = self.ser.in_waiting if hasattr(self.ser, "in_waiting") else 0
                data = self.ser.read(n or 1)
                if not data:
                    continue
                self.framer.processIncomingPacket(data, self._handle_request, unit=self.unit)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.ser.close()
            except Exception:
                pass

# -------------------- CLI / main --------------------
def main():
    ap = argparse.ArgumentParser(description="Modbus RTU slave with chaos + seq-from-HR0 (pymodbus 2.4 sync)")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--bytesize", type=int, choices=[7, 8], default=8)
    ap.add_argument("--parity", choices=["N", "E", "O"], default="N")
    ap.add_argument("--stopbits", type=int, choices=[1, 2], default=1)
    ap.add_argument("--unit", type=int, default=1)

    ap.add_argument("--coils", type=int, default=200)
    ap.add_argument("--discretes", type=int, default=200)
    ap.add_argument("--holdings", type=int, default=200)
    ap.add_argument("--inputs", type=int, default=200)

    # Sequence behavior (from HR[0])
    ap.add_argument("--seq-enable", action="store_true", default=True,
                    help="Enable injecting from HR[0] and incrementing/saving back")
    ap.add_argument("--seq-increment", type=int, default=1, help="Increment added after each injection")
    ap.add_argument("--seq-wrap", type=int, default=65536, help="Modulo for counter")

    # Chaos knobs
    ap.add_argument("--enable-chaos", action="store_true")
    ap.add_argument("--chaos-rate", type=int, default=100, help="0..100%% responses affected")
    ap.add_argument("--delay", type=int, default=0, help="Delay ms before sending selected responses")
    ap.add_argument("--split-at", type=int, default=0, help="Split after N bytes (0=off)")
    ap.add_argument("--split-gap", type=int, default=0, help="Gap ms between chunks")
    ap.add_argument("--mangle-prob", type=float, default=0.0, help="0..1 probability to corrupt selected responses")
    ap.add_argument("--mangle-crc", action="store_true", help="Corrupt CRC instead of payload")

    args = ap.parse_args()

    # Datastore
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [0] * args.discretes),
        co=ModbusSequentialDataBlock(0, [0] * args.coils),
        hr=ModbusSequentialDataBlock(0, [0] * args.holdings),
        ir=ModbusSequentialDataBlock(0, [0] * args.inputs),
        zero_mode=True,
    )
    context = ModbusServerContext(slaves={args.unit: store}, single=False)

    # Identity (informational only)
    identity = ModbusDeviceIdentification()
    identity.VendorName = "ChatGPT-Chaos"
    identity.ProductCode = "RTU"
    identity.VendorUrl = "https://github.com/riptideio/pymodbus"
    identity.ProductName = "Chaos RTU Test Server (sync v2.4)"
    identity.ModelName = "RTU-Chaos-24"
    identity.MajorMinorRevision = "2.4"

    # Apply seq + chaos cfg
    SEQ.enable    = bool(args.seq_enable)
    SEQ.increment = int(args.seq_increment)
    SEQ.wrap      = max(1, int(args.seq_wrap))
    CHAOS.enabled = args.enable_chaos
    CHAOS.rate_percent = max(0, min(100, args.chaos_rate))
    CHAOS.delay_ms = max(0, args.delay)
    CHAOS.split_at = max(0, args.split_at)
    CHAOS.split_gap_ms = max(0, args.split_gap)
    CHAOS.mangle_prob = max(0.0, min(1.0, args.mangle_prob))
    CHAOS.mangle_crc = bool(args.mangle_crc)

    # Ensure HR[0] exists
    if args.holdings < 1:
        raise SystemExit("Need --holdings >= 1 to store the sequence counter in HR[0].")

    print(f"[cfg] port={args.port} {args.baudrate}-{args.bytesize}{args.parity}{args.stopbits} unit={args.unit}")
    print(f"[cfg] seq-enable={SEQ.enable} inc={SEQ.increment} wrap={SEQ.wrap}")
    print(f"[cfg] chaos: enabled={CHAOS.enabled} rate={CHAOS.rate_percent}% "
          f"delay={CHAOS.delay_ms}ms split={CHAOS.split_at}/{CHAOS.split_gap_ms}ms "
          f"mangle_prob={CHAOS.mangle_prob} mangle_crc={CHAOS.mangle_crc}")

    server = RtuChaosServer(
        port=args.port, baudrate=args.baudrate, bytesize=args.bytesize,
        parity=args.parity, stopbits=args.stopbits, unit=args.unit,
        context=context, timeout=1.0
    )
    server.serve_forever()

if __name__ == "__main__":
    main()
