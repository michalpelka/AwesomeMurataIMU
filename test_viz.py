#!/usr/bin/env python3
"""
SCH16T IMU Visualizer
Packet format (29 bytes, packed, little-endian):
  uint8_t  sync   = 0xAA
  int32_t  rate[3]          (X, Y, Z)
  int32_t  acc[3]           (X, Y, Z)
  int32_t  temp
"""

import struct
import serial
import serial.tools.list_ports
import argparse
import collections
import threading
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Packet definition ──────────────────────────────────────────────────────────
SYNC_BYTE    = 0xAA
PKT_FMT      = '<BQ3i3ii'  # uint8 sync + uint64 timestamp_us + 3×rate + 3×acc + temp

import datetime
PKT_SIZE     = struct.calcsize(PKT_FMT)  # 29 bytes

# ── Sensitivity (must match config.h) ─────────────────────────────────────────
SENSITIVITY_RATE = 1600.0   # LSB / dps  (20-bit mode)
SENSITIVITY_ACC  = 3200.0   # LSB / m/s²

# ── Plot config ────────────────────────────────────────────────────────────────
HISTORY = 500   # samples to show


# ── Serial reader thread ───────────────────────────────────────────────────────
class IMUReader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=1)
        self.lock = threading.Lock()

        n = HISTORY
        self.rate = [collections.deque([0.0]*n, maxlen=n) for _ in range(3)]
        self.acc  = [collections.deque([0.0]*n, maxlen=n) for _ in range(3)]
        self.temp = collections.deque([0.0]*n, maxlen=n)
        self.pkt_count = 0
        self.err_count = 0
        self.timestamp_us = 0

    def run(self):
        buf = b''
        while True:
            buf += self.ser.read(PKT_SIZE * 4)

            # sync on 0xAA byte
            while len(buf) >= PKT_SIZE:
                idx = buf.find(bytes([SYNC_BYTE]))
                if idx == -1:
                    buf = b''
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < PKT_SIZE:
                    break

                raw = buf[:PKT_SIZE]
                buf = buf[PKT_SIZE:]  # advance past full packet

                try:
                    fields = struct.unpack(PKT_FMT, raw)
                except struct.error:
                    self.err_count += 1
                    continue

                sync, timestamp_us, rx, ry, rz, ax, ay, az, temp_raw = fields
                if sync != SYNC_BYTE:
                    self.err_count += 1
                    continue

                # convert to physical units
                rate_dps = [rx / SENSITIVITY_RATE,
                            ry / SENSITIVITY_RATE,
                            rz / SENSITIVITY_RATE]
                acc_ms2  = [ax / SENSITIVITY_ACC,
                            ay / SENSITIVITY_ACC,
                            az / SENSITIVITY_ACC]
                temp_c   = temp_raw / 100.0

                with self.lock:
                    for i in range(3):
                        self.rate[i].append(rate_dps[i])
                        self.acc[i].append(acc_ms2[i])
                    self.temp.append(temp_c)
                    self.timestamp_us = timestamp_us
                    self.pkt_count += 1


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='SCH16T IMU Visualizer')
    parser.add_argument('--port', default=None, help='Serial port (e.g. /dev/ttyUSB0 or COM3)')
    parser.add_argument('--baud', type=int, default=460800)
    parser.add_argument('--console', action='store_true', help='Print data to console instead of plotting')
    args = parser.parse_args()

    # auto-detect port
    if args.port is None:
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found. Use --port")
            return
        args.port = ports[0].device
        print(f"Auto-selected port: {args.port}")

    print(f"Connecting to {args.port} @ {args.baud} baud  (packet size {PKT_SIZE} bytes)")
    reader = IMUReader(args.port, args.baud)
    reader.start()

    if args.console:
        last_count = 0
        while True:
            time.sleep(0.1)
            with reader.lock:
                cnt = reader.pkt_count
                if cnt == last_count:
                    continue
                last_count = cnt
                ts  = reader.timestamp_us
                rx  = reader.rate[0][-1]
                ry  = reader.rate[1][-1]
                rz  = reader.rate[2][-1]
                ax  = reader.acc[0][-1]
                ay  = reader.acc[1][-1]
                az  = reader.acc[2][-1]
                tmp = reader.temp[-1]
            dt = datetime.datetime.utcfromtimestamp(ts / 1e6) if ts else datetime.datetime.utcfromtimestamp(0)
            print(f"[{dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}]  "
                  f"rate: {rx:+8.3f} {ry:+8.3f} {rz:+8.3f} °/s  "
                  f"acc: {ax:+7.3f} {ay:+7.3f} {az:+7.3f} m/s²  "
                  f"temp: {tmp:+6.2f} °C  "
                  f"pkts: {cnt}")
        return

    # ── Figure layout ──────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(12, 8))
    fig.suptitle('SCH16T IMU', fontsize=13)

    colors_xyz = ['#e74c3c', '#2ecc71', '#3498db']
    labels_xyz = ['X', 'Y', 'Z']

    ax_rate, ax_acc, ax_temp = axes
    ax_rate.set_title('Angular Rate (°/s)')
    ax_acc.set_title('Acceleration (m/s²)')
    ax_temp.set_title('Temperature (°C)')

    for ax in (ax_rate, ax_acc):
        ax.set_xlim(0, HISTORY)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left')

    ax_temp.set_xlim(0, HISTORY)
    ax_temp.grid(True, alpha=0.3)

    lines_rate = [ax_rate.plot([], [], color=colors_xyz[i], label=labels_xyz[i], lw=1)[0] for i in range(3)]
    lines_acc  = [ax_acc.plot([],  [], color=colors_xyz[i], label=labels_xyz[i], lw=1)[0] for i in range(3)]
    line_temp, = ax_temp.plot([], [], color='#f39c12', lw=1)

    ax_rate.legend(loc='upper left')
    ax_acc.legend(loc='upper left')

    title_text = fig.text(0.5, 0.96, '', ha='center', fontsize=10, color='gray')

    def update(_):
        with reader.lock:
            rate_snap = [list(reader.rate[i]) for i in range(3)]
            acc_snap  = [list(reader.acc[i])  for i in range(3)]
            temp_snap = list(reader.temp)
            cnt = reader.pkt_count

        x = range(len(temp_snap))

        for i in range(3):
            lines_rate[i].set_data(x, rate_snap[i])
            lines_acc[i].set_data(x, acc_snap[i])

        line_temp.set_data(x, temp_snap)

        # auto-scale Y axes
        for ax, data_list in [(ax_rate, rate_snap), (ax_acc, acc_snap)]:
            all_vals = [v for d in data_list for v in d]
            if all_vals:
                mn, mx = min(all_vals), max(all_vals)
                pad = max(abs(mx - mn) * 0.1, 0.01)
                ax.set_ylim(mn - pad, mx + pad)

        if temp_snap:
            mn, mx = min(temp_snap), max(temp_snap)
            pad = max(abs(mx - mn) * 0.1, 0.5)
            ax_temp.set_ylim(mn - pad, mx + pad)

        title_text.set_text(f'packets received: {cnt}')
        return lines_rate + lines_acc + [line_temp, title_text]

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()


if __name__ == '__main__':
    main()