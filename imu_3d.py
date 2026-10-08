#!/usr/bin/env python3
"""
SCH16T 3D orientation demo.

Shows a box that follows the board's orientation. Orientation comes from a
Mahony filter: the gyro is integrated every sample and the accelerometer pulls
roll and pitch back toward gravity. There is no magnetometer, so yaw drifts
slowly; press 'r' to reset it.

Keep the board still for the first 2 s while the gyro bias is measured.

  python3 imu_3d.py --port /dev/ttyACM1
"""

import argparse
import math
import struct
import threading

import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from test_viz import (PKT_FMT, PKT_SIZE, SYNC_BYTE, SENSITIVITY_RATE,
                      SENSITIVITY_ACC, crc8, gps_status_text)

BIAS_SECONDS = 2.0   # board must be still while the gyro bias is measured
KP           = 1.0   # accelerometer correction gain (higher: less drift, more vibration)
MAX_DT       = 0.01  # s; larger timestamp steps (GPS lock, dropped data) are ignored


class OrientationReader(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, timeout=1)
        self.lock = threading.Lock()
        self.q = [1.0, 0.0, 0.0, 0.0]         # w, x, y, z: body to world
        self.bias = [0.0, 0.0, 0.0]           # rad/s
        self.bias_sum = [0.0, 0.0, 0.0]
        self.bias_n = 0
        self.bias_start_us = None
        self.calibrated = False
        self.last_us = None
        self.gps_status = 0
        self.pkt_count = 0
        self.rate_hz = 0.0
        self._rate_start = None

    def reset(self):
        with self.lock:
            self.q = [1.0, 0.0, 0.0, 0.0]

    def run(self):
        buf = b''
        while True:
            buf += self.ser.read(self.ser.in_waiting or 1)
            while True:
                i = buf.find(bytes([SYNC_BYTE]))
                if i < 0:
                    buf = b''
                    break
                if len(buf) - i < PKT_SIZE:
                    buf = buf[i:]
                    break
                raw = buf[i:i + PKT_SIZE]
                if crc8(raw[:-1]) != raw[-1]:
                    buf = buf[i + 1:]
                    continue
                buf = buf[i + PKT_SIZE:]
                self.process(struct.unpack(PKT_FMT, raw))

    def process(self, fields):
        _, ts, rx, ry, rz, ax, ay, az, _, gps_status, _ = fields
        gyro = [math.radians(v / SENSITIVITY_RATE) for v in (rx, ry, rz)]
        acc = [v / SENSITIVITY_ACC for v in (ax, ay, az)]

        if self._rate_start is None:
            self._rate_start = (ts, self.pkt_count)
        elif ts - self._rate_start[0] > 1_000_000:
            self.rate_hz = (self.pkt_count - self._rate_start[1]) * 1e6 / (ts - self._rate_start[0])
            self._rate_start = (ts, self.pkt_count)

        dt = None if self.last_us is None else (ts - self.last_us) * 1e-6
        self.last_us = ts

        with self.lock:
            self.gps_status = gps_status
            self.pkt_count += 1
            if not self.calibrated:
                self.collect_bias(ts, gyro)
                return
            if dt is None or not 0 < dt <= MAX_DT:
                return
            self.q = mahony(self.q, [g - b for g, b in zip(gyro, self.bias)], acc, dt)

    def collect_bias(self, ts, gyro):
        if self.bias_start_us is None:
            self.bias_start_us = ts
        for k in range(3):
            self.bias_sum[k] += gyro[k]
        self.bias_n += 1
        if ts - self.bias_start_us >= BIAS_SECONDS * 1e6:
            self.bias = [s / self.bias_n for s in self.bias_sum]
            self.calibrated = True


def mahony(q, gyro, acc, dt):
    """One Mahony update (proportional term only). Returns the new quaternion."""
    w, x, y, z = q
    gx, gy, gz = gyro
    n = math.sqrt(acc[0] ** 2 + acc[1] ** 2 + acc[2] ** 2)
    if n > 0:
        ax, ay, az = (a / n for a in acc)
        # gravity direction (world up) as seen in the body frame
        vx = 2 * (x * z - w * y)
        vy = 2 * (w * x + y * z)
        vz = w * w - x * x - y * y + z * z
        # error between measured and estimated gravity
        gx += KP * (ay * vz - az * vy)
        gy += KP * (az * vx - ax * vz)
        gz += KP * (ax * vy - ay * vx)
    h = 0.5 * dt
    w, x, y, z = (w + h * (-x * gx - y * gy - z * gz),
                  x + h * ( w * gx + y * gz - z * gy),
                  y + h * ( w * gy - x * gz + z * gx),
                  z + h * ( w * gz + x * gy - y * gx))
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return [w / n, x / n, y / n, z / n]


def rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def euler_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


# Box shaped like a board: 2 x 1.2 x 0.3, centered on the origin
L, W, H = 1.0, 0.6, 0.15
VERTICES = np.array([[sx * L, sy * W, sz * H]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
FACE_COLORS = ['#7f8c8d', '#7f8c8d', '#3498db', '#3498db', '#e74c3c', '#e74c3c']
AXIS_COLORS = ['#e74c3c', '#2ecc71', '#3498db']


def main():
    parser = argparse.ArgumentParser(description='SCH16T 3D orientation demo')
    parser.add_argument('--port', required=True, help='Serial port, e.g. /dev/ttyACM1')
    args = parser.parse_args()

    reader = OrientationReader(args.port)
    reader.start()

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    box = Poly3DCollection([], edgecolor='black', linewidths=0.8, alpha=0.9)
    ax.add_collection3d(box)
    axes_lines = [ax.plot([], [], [], color=c, lw=2.5)[0] for c in AXIS_COLORS]
    info = fig.text(0.02, 0.97, '', va='top', family='monospace', fontsize=10)
    fig.text(0.02, 0.02, "r: reset yaw/orientation   q: quit", fontsize=9, color='gray')

    def on_key(event):
        if event.key == 'r':
            reader.reset()

    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_):
        with reader.lock:
            q = list(reader.q)
            calibrated = reader.calibrated
            gps = reader.gps_status
            rate = reader.rate_hz

        R = rotation_matrix(q)
        verts = VERTICES @ R.T
        box.set_verts([[verts[i] for i in face] for face in FACES])
        box.set_facecolor(FACE_COLORS)
        for k, line in enumerate(axes_lines):
            tip = R[:, k] * 1.1
            line.set_data_3d([0, tip[0]], [0, tip[1]], [0, tip[2]])

        if not calibrated:
            status = 'Keep the board still: measuring gyro bias...'
        else:
            roll, pitch, yaw = euler_deg(q)
            status = f'roll {roll:+7.1f}°   pitch {pitch:+7.1f}°   yaw {yaw:+7.1f}°'
        info.set_text(f'{status}\n{rate:6.0f} Hz   {gps_status_text(gps)}')
        return [box, info] + axes_lines

    ani = animation.FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
