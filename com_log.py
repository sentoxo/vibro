"""Read a COM port and write each line with a timestamp to a txt file.

Usage:
    python com_log.py --port COM3 --baud 921600 --out log.txt
    python com_log.py --port COM3 --lines 1000 --silent
"""
import argparse
import serial
import time


def main():
    p = argparse.ArgumentParser(description="Log COM port lines with timestamps")
    p.add_argument("--port", required=True, help="COM port, e.g. COM3")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--out", default="log.txt", help="output txt file")
    p.add_argument("--lines", type=int, default=0,
                   help="max lines to read (0 = until Ctrl-C)")
    p.add_argument("--silent", action="store_true",
                   help="do not print lines to the console")
    args = p.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    if not args.silent:
        print(f"Opening {args.port} @ {args.baud} baud ... Press Ctrl-C to stop.")

    try:
        with open(args.out, "w") as f:
            count = 0
            while args.lines <= 0 or count < args.lines:
                line = ser.readline()
                if not line:
                    continue
                text = line.decode("ascii", errors="ignore").strip()
                if not text:
                    continue
                ts = f"{time.time():.6f}"
                if not args.silent:
                    print(f"{ts}:{text}")
                f.write(f"{ts}:{text}\n")
                f.flush()
                count += 1
        if not args.silent:
            print(f"Done. Wrote {count} lines to {args.out}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
