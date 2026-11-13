#!/usr/bin/env python3
"""Person detector for the C21 vision behaviours - runs on the Jetson HOST (not in c21_robot).

The ROS container has no GPU access; the host has CUDA torch + ultralytics already installed, so
this small server uses them as they are (nothing is installed or changed on the host).

Protocol (TCP 127.0.0.1:5577, the container uses host networking), one request per frame:
    request : b'C21F' + uint16 width + uint16 height (big endian) + width*height*3 bytes RGB
    response: uint32 length + JSON {"boxes": [[x1, y1, x2, y2, conf], ...], "ms": inference ms}
The model is loaded on the first request and freed after IDLE_S without requests, so it costs
nothing while no vision mode runs.

    python3 yolo_server.py [--model ~/yolov8n.pt] [--port 5577]
"""
import argparse
import json
import os
import socket
import struct
import threading
import time

import numpy as np

IDLE_S = 120.0


class Detector:
    def __init__(self, path, conf, imgsz):
        self.path, self.conf, self.imgsz = path, conf, imgsz
        self.model = None
        self.used = 0.0
        self.lock = threading.Lock()

    def detect(self, rgb):
        with self.lock:
            if self.model is None:
                from ultralytics import YOLO
                t = time.time()
                self.model = YOLO(self.path)
                self.model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=self.imgsz, half=True,
                                   classes=[0], verbose=False)        # warm-up (CUDA init)
                print(f'model {self.path} loaded in {time.time() - t:.1f} s', flush=True)
            self.used = time.monotonic()
            t = time.perf_counter()
            # ultralytics expects BGR for numpy input
            r = self.model.predict(rgb[:, :, ::-1], imgsz=self.imgsz, conf=self.conf, half=True,
                                   classes=[0], verbose=False)[0]
            ms = (time.perf_counter() - t) * 1000
            b = r.boxes
            boxes = np.concatenate([b.xyxy.cpu().numpy(), b.conf.cpu().numpy()[:, None]], axis=1) \
                if len(b) else np.zeros((0, 5))
            return [[round(float(v), 1) for v in row[:4]] + [round(float(row[4]), 3)] for row in boxes], ms

    def unload_if_idle(self):
        with self.lock:
            if self.model is not None and time.monotonic() - self.used > IDLE_S:
                self.model = None
                import torch
                torch.cuda.empty_cache()
                print('idle: model unloaded', flush=True)


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(min(1 << 20, n - len(buf)))
        if not chunk:
            raise ConnectionError('closed')
        buf += chunk
    return bytes(buf)


def serve(conn, det):
    with conn:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        while True:
            head = recv_exact(conn, 8)
            if head[:4] != b'C21F':
                raise ConnectionError('bad header')
            w, h = struct.unpack('>HH', head[4:])
            if not (0 < w <= 1920 and 0 < h <= 1080):
                raise ConnectionError('bad size')
            rgb = np.frombuffer(recv_exact(conn, w * h * 3), np.uint8).reshape(h, w, 3)
            try:
                boxes, ms = det.detect(rgb)
                out = {'boxes': boxes, 'ms': round(ms, 1)}
            except Exception as e:  # noqa: BLE001 - report to the client, keep serving
                out = {'boxes': [], 'error': str(e)}
            data = json.dumps(out).encode()
            conn.sendall(struct.pack('>I', len(data)) + data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=os.path.expanduser('~/yolov8n.pt'))
    ap.add_argument('--port', type=int, default=5577)
    ap.add_argument('--conf', type=float, default=0.4)
    ap.add_argument('--imgsz', type=int, default=640)
    a = ap.parse_args()
    det = Detector(a.model, a.conf, a.imgsz)

    def idle_loop():
        while True:
            time.sleep(10)
            det.unload_if_idle()
    threading.Thread(target=idle_loop, daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', a.port))
    srv.listen(4)
    print(f'yolo server on 127.0.0.1:{a.port}, model {a.model} (loaded on first request)', flush=True)
    while True:
        conn, _ = srv.accept()

        def run(c=conn):
            try:
                serve(c, det)
            except (ConnectionError, OSError):
                pass
        threading.Thread(target=run, daemon=True).start()


if __name__ == '__main__':
    main()
