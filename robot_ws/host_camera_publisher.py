#!/usr/bin/env python3
import subprocess
import signal
import sys
import time

running = True
proc = None

def shutdown(sig, frame):
    global running, proc
    print("\n🛑 Stopping camera stream...")
    running = False
    if proc is not None:
        proc.terminate()
    sys.exit(0)

def main():
    global proc

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    cmd = [
        "rpicam-vid",
        "-t", "0",
        "-n",
        "--listen",
        "--codec", "mjpeg",
        "--width", "1280",
        "--height", "720",
        "--framerate", "30",
        "-o", "tcp://0.0.0.0:8888",
    ]

    while running:
        print("🚀 Starting Arducam MJPEG TCP stream...")
        print("📡 Waiting for Docker client on tcp://127.0.0.1:8888")

        proc = subprocess.Popen(cmd)
        proc.wait()

        if running:
            print("⚠️ Stream client disconnected. Restarting camera stream...")
            time.sleep(1)

if __name__ == "__main__":
    main()