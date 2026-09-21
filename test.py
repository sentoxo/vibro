"""Standalone COM-port packet diagnostic.

Opens the serial port, counts packets, detects lost packets via the `seq`
field, and prints incoming frequency (packets/sec) + losses to the terminal.

Usage:
    python com_diag.py --port COM3 --baud 921600 --duration 30

Compare its "lost" count with the app's. If this script also loses packets,
the stream/ESP is at fault. If it stays clean, the PyQt app drops them.
"""
import argparse
import re
import serial
import serial.tools.list_ports
import time

# Expected line: S<sid>,<seq>,<x>,<y>,<z>
SEQ_LINE = re.compile(r"^S(\d),(\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")


def list_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    print("Available ports:", ports)
    return ports


def main():
    p = argparse.ArgumentParser(description="COM-port packet loss diagnostic")
    p.add_argument("--port", help="COM port, e.g. COM3 (omit to list and pick)")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--duration", type=float, default=30.0,
                   help="run seconds (0 = run until Ctrl-C)")
    args = p.parse_args()

    port = args.port
    if not port:
        ports = list_ports()
        if not ports:
            print("No ports found.")
            return
        port = input(f"Select port from {ports}: ").strip()

    print(f"Opening {port} @ {args.baud} baud ...")
    ser = serial.Serial(port, args.baud, timeout=0.1)
    print("Connected. Press Ctrl-C to stop.\n")

    start = time.time()
    packets = 0
    lost = 0
    last_seq = None
    window_start = start
    window_count = 0

    try:
        while True:
            elapsed = time.time() - start
            if 0 < args.duration <= elapsed:
                break

            line = ser.readline()
            if not line:
                continue
            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not text:
                continue

            m = SEQ_LINE.match(text)
            if not m:
                # Corrupted / unparseable line (read-stage loss).
                packets += 1
                continue

            seq = int(m.group(2))
            packets += 1

            if last_seq is not None and seq - last_seq > 1:
                lost += seq - last_seq - 1
            last_seq = seq

            # Per-second stats.
            if time.time() - window_start >= 1.0:
                dt = time.time() - window_start
                freq = window_count / dt if dt > 0 else 0
                print(f"[t={elapsed:6.1f}s] "
                      f"incoming={freq:8.1f} Hz  "
                      f"total_packets={packets:>10d}  "
                      f"lost={lost:>10d}  "
                      f"last_seq={last_seq}")
                window_start = time.time()
                window_count = 0
            window_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        dt = time.time() - start
        print("\n--- Summary ---")
        print(f"total packets received : {packets}")
        print(f"lost packets (seq gaps): {lost}")
        print(f"avg incoming frequency : {packets / dt:.1f} Hz" if dt > 0 else "")
        ser.close()


if __name__ == "__main__":
    main()
