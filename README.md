
# Modbus Chaos Test Kit (pymodbus 2.4, sync)

This bundle includes:
- `modbus_rtu_chaos_server_sync_v24.py` — RTU slave with chaos (delay/split/mangle), and **sequence injection sourced from HR[0]**. After each read response, it reads HR[0], injects the value, **increments**, and saves back.
- `modbus_chaos_client_sync_v24.py` — Client for TCP / RTU-over-TCP / Serial RTU with chaos on outbound, **sequence verification**, **speed test**, and an option to **seed** HR[0] before tests via Modbus write.

## Install
```bash
pip install "pymodbus==2.4.*" pyserial
```

## Quick start
Start the server (RTU on /dev/ttyUSB0, 19200-8E1):
```bash
python3 modbus_rtu_chaos_server_sync_v24.py --port /dev/ttyUSB0 --baudrate 19200 --parity E --stopbits 1
```

Seed HR[0] to `0x1234`, then speed test holding-register reads over TCP:
```bash
python3 modbus_chaos_client_sync_v24.py --mode tcp --host 192.168.1.50 --port 502   --seed-seq 0x1234 --seed-seq-addr 0   --op read-holding --addr 0 --count 2 --unit 1   --verify-seq --verify-seq-start 0x1234 --verify-seq-increment 1   --speed 500 --quiet --timeout 0.5
```

See the inline `--help` for all options.
