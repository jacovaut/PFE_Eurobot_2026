#!/usr/bin/env python3
import subprocess
import signal
import sys

def main():
    cmd = [
        "rpicam-vid",
        "-t", "0",                 # run forever
        "-n",                      # no preview window
        "--inline",                # required for streaming
        "--listen",                # act as server
        "-o", "tcp://0.0.0.0:8888"
    ]

    print("🚀 Starting Arducam TCP stream...")
    print("📡 Stream available at tcp://127.0.0.1:8888")

    proc = subprocess.Popen(cmd)

    def shutdown(sig, frame):
        print("\n🛑 Stopping camera stream...")
        proc.terminate()
        proc.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    proc.wait()


if __name__ == "__main__":
    main()