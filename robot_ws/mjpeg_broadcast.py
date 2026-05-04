#!/usr/bin/env python3
"""
MJPEG broadcast proxy.

Reads from rpicam-vid on UPSTREAM_PORT (single-client raw TCP MJPEG),
then re-serves the stream in two ways:
  - Raw TCP on RAW_PORT   (port 8888) — for local ROS nodes
  - HTTP MJPEG on HTTP_PORT (port 8080) — for browsers / VLC over WiFi
        http://<pi-ip>:8080/
"""

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

UPSTREAM_HOST = "127.0.0.1"
UPSTREAM_PORT = 8887   # rpicam-vid output port

RAW_HOST  = "0.0.0.0"
RAW_PORT  = 8888       # raw TCP MJPEG for ROS nodes

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080       # HTTP MJPEG for browsers / VLC


# ──────────────────────────────────────────────
# Shared latest frame (bytes of a complete JPEG)
# ──────────────────────────────────────────────
_frame_lock   = threading.Lock()
_latest_frame: bytes | None = None

# Raw TCP client list
_tcp_clients: list[socket.socket] = []
_tcp_lock = threading.Lock()


# ──────────────────────────────────────────────
# Upstream reader — runs forever, reconnects
# ──────────────────────────────────────────────
def upstream_loop():
    global _latest_frame
    while True:
        upstream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            upstream.connect((UPSTREAM_HOST, UPSTREAM_PORT))
            print(f"[PROXY] Connected to upstream {UPSTREAM_HOST}:{UPSTREAM_PORT}")
        except (ConnectionRefusedError, OSError) as e:
            print(f"[PROXY] Upstream not ready ({e}), retrying in 1s...")
            upstream.close()
            time.sleep(1.0)
            continue

        buf = b""
        try:
            while True:
                chunk = upstream.recv(65536)
                if not chunk:
                    print("[PROXY] Upstream closed, reconnecting...")
                    break

                buf += chunk

                while True:
                    start = buf.find(b"\xff\xd8")
                    end   = buf.find(b"\xff\xd9")
                    if start == -1 or end == -1 or end <= start:
                        break
                    frame = buf[start:end + 2]
                    buf   = buf[end + 2:]

                    # update shared frame for HTTP clients
                    with _frame_lock:
                        _latest_frame = frame

                    # forward to raw TCP clients (ROS nodes)
                    with _tcp_lock:
                        dead = []
                        for c in _tcp_clients:
                            try:
                                c.sendall(frame)
                            except OSError:
                                dead.append(c)
                        for c in dead:
                            _tcp_clients.remove(c)
                            try:
                                c.close()
                            except OSError:
                                pass

                if len(buf) > 4_000_000:
                    buf = buf[-500_000:]

        except OSError as e:
            print(f"[PROXY] Upstream error: {e}")
        finally:
            upstream.close()


# ──────────────────────────────────────────────
# Raw TCP accept loop
# ──────────────────────────────────────────────
def raw_accept_loop():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((RAW_HOST, RAW_PORT))
    srv.listen(8)
    print(f"[PROXY] Raw TCP listening on {RAW_HOST}:{RAW_PORT}")
    while True:
        conn, addr = srv.accept()
        print(f"[PROXY] Raw client connected: {addr}")
        with _tcp_lock:
            _tcp_clients.append(conn)


# ──────────────────────────────────────────────
# HTTP MJPEG server
# ──────────────────────────────────────────────
BOUNDARY = b"mjpegboundary"

class MjpegHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # suppress per-request logs

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        print(f"[HTTP] Client connected: {self.client_address}")
        try:
            while True:
                with _frame_lock:
                    frame = _latest_frame

                if frame is None:
                    time.sleep(0.02)
                    continue

                header = (
                    f"--{BOUNDARY.decode()}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n"
                    f"\r\n"
                ).encode()

                self.wfile.write(header + frame + b"\r\n")
                self.wfile.flush()
                time.sleep(0.033)  # ~30 fps cap
        except (BrokenPipeError, ConnectionResetError):
            pass
        print(f"[HTTP] Client disconnected: {self.client_address}")


def http_server_loop():
    server = HTTPServer((HTTP_HOST, HTTP_PORT), MjpegHandler)
    print(f"[HTTP] MJPEG server on http://{HTTP_HOST}:{HTTP_PORT}/")
    server.serve_forever()


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────
if __name__ == "__main__":
    threading.Thread(target=raw_accept_loop, daemon=True).start()
    threading.Thread(target=http_server_loop, daemon=True).start()
    upstream_loop()  # blocks, reconnects forever
