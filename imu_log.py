#!/usr/bin/env python3
"""
SCH16T IMU logger: writes every packet to a CSV file in HDMapping's IMU
format (https://github.com/MapsHD/HDMapping/wiki/File-Formats):

  timestamp gyroX gyroY gyroZ accX accY accZ imuId timestampUnix

Space-separated, timestamps in ns, gyro in rad/s, acceleration in g.
timestamp is the device clock (Unix time with GPS lock, otherwise time since
boot); timestampUnix is the host's clock when the packet was received.

Stop with Ctrl-C, or use --duration.
"""

import argparse
import csv
import datetime
import glob
import math
import struct
import sys
import time

import serial

# ── Packet definition (must match main.c, see test_viz.py) ────────────────────
SYNC_BYTE = 0xAA
PKT_FMT   = '<BQ3i3iiBB'
PKT_SIZE  = struct.calcsize(PKT_FMT)

# ── Sensitivity (must match config.h) ─────────────────────────────────────────
SENSITIVITY_RATE = 1600.0   # LSB / dps  (20-bit mode)
SENSITIVITY_ACC  = 3200.0   # LSB / m/s²

G = 9.80665                 # m/s² per g

GYRO_K = math.radians(1.0) / SENSITIVITY_RATE   # counts -> rad/s
ACC_K  = 1.0 / (SENSITIVITY_ACC * G)            # counts -> g

# Packets older than this gap are treated as stale data buffered before the
# port was opened (see "Known limitations" in README.md).
MAX_GAP_US      = 5000
CONTINUOUS_PKTS = 100   # > the 52 packets the board can hold


def _crc8_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07 if crc & 0x80 else crc << 1) & 0xFF
        table.append(crc)
    return table

CRC8_TABLE = _crc8_table()

def crc8(data):
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc


def packets(ser):
    """Yield valid decoded packets, resyncing byte by byte on errors."""
    buf = bytearray()
    while True:
        buf += ser.read(max(ser.in_waiting, PKT_SIZE))
        while len(buf) >= PKT_SIZE:
            if buf[0] != SYNC_BYTE:
                idx = buf.find(SYNC_BYTE)
                del buf[:idx if idx != -1 else len(buf)]
                continue
            raw = bytes(buf[:PKT_SIZE])
            if crc8(raw[:-1]) != raw[-1]:
                packets.errors += 1
                del buf[:1]
                continue
            del buf[:PKT_SIZE]
            yield struct.unpack(PKT_FMT, raw)

packets.errors = 0


def default_port():
    ports = glob.glob('/dev/serial/by-id/*STM32_Virtual_ComPort*')
    return ports[0] if ports else None


def main():
    parser = argparse.ArgumentParser(description='SCH16T IMU CSV logger')
    parser.add_argument('--port', default=default_port(),
                        help='Serial port (default: the STM32 virtual COM port)')
    parser.add_argument('--baud', type=int, default=460800,
                        help='Baud rate (ignored for USB CDC)')
    parser.add_argument('-o', '--output',
                        help='CSV file (default: imu_YYYYmmdd_HHMMSS.csv)')
    parser.add_argument('--duration', type=float,
                        help='Stop after this many seconds')
    parser.add_argument('--imu-id', type=int, default=0,
                        help='imuId column (default 0)')
    args = parser.parse_args()

    if not args.port:
        sys.exit('No STM32 virtual COM port found; pass --port')
    out = args.output or datetime.datetime.now().strftime('imu_%Y%m%d_%H%M%S.csv')

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    ser.reset_input_buffer()

    with open(out, 'w', newline='', buffering=1 << 16) as f:
        w = csv.writer(f, delimiter=' ')
        w.writerow(['timestamp', 'gyroX', 'gyroY', 'gyroZ',
                    'accX', 'accY', 'accZ', 'imuId', 'timestampUnix'])
        print(f'Logging {args.port} -> {out}  (Ctrl-C to stop)', file=sys.stderr)

        start = time.monotonic()
        last_report = start
        count = gaps = 0
        last_ts = None
        run = 0          # consecutive packets with small gaps while syncing
        synced = False
        try:
            for (_, ts, rx, ry, rz, ax, ay, az, _, gps, _) in packets(ser):
                dt = ts - last_ts if last_ts is not None else 0
                last_ts = ts
                if not synced:
                    # Drop stale buffered packets until timestamps are continuous
                    run = run + 1 if 0 < dt < MAX_GAP_US else 0
                    synced = run >= CONTINUOUS_PKTS
                    continue
                if not 0 < dt < MAX_GAP_US:
                    gaps += 1

                w.writerow([ts * 1000,
                            f'{rx * GYRO_K:.9f}', f'{ry * GYRO_K:.9f}', f'{rz * GYRO_K:.9f}',
                            f'{ax * ACC_K:.9f}', f'{ay * ACC_K:.9f}', f'{az * ACC_K:.9f}',
                            args.imu_id, time.time_ns()])
                count += 1

                now = time.monotonic()
                if now - last_report >= 1.0:
                    print(f'\r{count} pkts  {count / (now - start):.0f} Hz  '
                          f'crc err {packets.errors}  gaps {gaps}  gps 0x{gps:02X}   ',
                          end='', file=sys.stderr)
                    last_report = now
                    if args.duration and now - start >= args.duration:
                        break
        except KeyboardInterrupt:
            pass

    print(f'\nWrote {count} packets to {out}', file=sys.stderr)


if __name__ == '__main__':
    main()
