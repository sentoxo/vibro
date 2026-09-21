"""Sequential (seq) integrity analyzer for com_log output.

Reads a timestamped COM-port log written by com_log:

    <timestamp>:S<sid>,<seq>,<x>,<y>,<z>

and reports packet integrity from the <seq> field. It assumes seq should
increment by exactly 1 on every line, so:

    delta ==  1   -> ok (no loss)
    delta >  1    -> lost (delta - 1) packets
    delta ==  0   -> duplicate / stale read (seq did not advance)
    delta <  0    -> seq reset / counter went backwards

Usage:
    python seq_analyzer.py                 # reads log.txt
    python seq_analyzer.py --file log.txt
"""
import argparse
import re

# timestamp : S<sid>,<seq>,<x>,<y>,<z>
LINE_RE = re.compile(r"^([0-9.]+):S(\d+),(\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")


def analyze(path):
    ts = []
    seqs = []
    malformed = 0

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = LINE_RE.match(line)
            if not m:
                malformed += 1
                continue
            ts.append(float(m.group(1)))
            seqs.append(int(m.group(3)))

    if not seqs:
        print(f"No parseable lines found in {path!r}.")
        return

    # ---- seq integrity -------------------------------------------------
    lost = 0
    duplicates = 0
    resets = 0
    gaps = []  # (index, prev_seq, cur_seq, delta, lost_count)

    for i in range(1, len(seqs)):
        prev, cur = seqs[i - 1], seqs[i]
        d = cur - prev
        if d == 1:
            continue
        if d > 1:
            n_lost = d - 1
            lost += n_lost
            gaps.append((i, prev, cur, d, n_lost))
        elif d == 0:
            duplicates += 1
        else:
            resets += 1

    expected_last = seqs[0] + len(seqs) - 1
    total_lost = sum(g[4] for g in gaps)

    # ---- timing --------------------------------------------------------
    if len(ts) >= 2:
        duration = ts[-1] - ts[0]
        intervals = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        mean_iv = sum(intervals) / len(intervals)
        median_iv = sorted(intervals)[len(intervals) // 2]
        max_iv = max(intervals)
        # statistical outlier threshold: mean + 3*stddev catches true stalls
        # above the stream's normal jitter.
        variance = sum((x - mean_iv) ** 2 for x in intervals) / len(intervals)
        std_iv = variance ** 0.5
        threshold = mean_iv + 3.0 * std_iv
        big_gaps = sorted(
            ((i, intervals[i]) for i in range(len(intervals))
             if intervals[i] > threshold),
            key=lambda t: t[1],
            reverse=True,
        )
    else:
        duration = mean_iv = median_iv = max_iv = 0.0
        threshold = 0.0
        big_gaps = []

    # ---- report --------------------------------------------------------
    print("=" * 60)
    print(f"seq integrity report  ({path})")
    print("=" * 60)
    print(f"lines parsed         : {len(seqs)}")
    print(f"malformed lines      : {malformed}")
    print(f"seq first / last     : {seqs[0]} / {seqs[-1]}")
    print(f"seq expected (last)  : {expected_last}  (+1 per line)")
    print(f"seq received (lines) : {len(seqs)}")
    print()
    print(f"packets lost (gaps)  : {total_lost}")
    print(f"  gap events (>1)    : {len(gaps)}")
    print(f"duplicates (delta 0) : {duplicates}")
    print(f"resets  (delta < 0)  : {resets}")
    print()

    if duration > 0:
        print("-" * 60)
        print("timing")
        print("-" * 60)
        print(f"duration             : {duration:.3f} s")
        print(f"effective rate       : {len(seqs) / duration:.1f} lines/s")
        print(f"mean interval        : {mean_iv * 1000:.3f} ms")
        print(f"median interval      : {median_iv * 1000:.3f} ms")
        print(f"max interval         : {max_iv * 1000:.3f} ms")
        print(f"std dev interval     : {std_iv * 1000:.3f} ms")
        print(f"outlier threshold    : {threshold * 1000:.2f} ms (mean+3*std)")
        if big_gaps:
            shown = big_gaps[:20]
            print(f"timing outliers > threshold ({len(big_gaps)} total, "
                  f"showing top {len(shown)}):")
            for i, iv in shown:
                print(f"  line {i} -> {i + 1}: {iv * 1000:.2f} ms")
            if len(big_gaps) > len(shown):
                print(f"  ... and {len(big_gaps) - len(shown)} more")
        else:
            print("timing outliers: none")

    # ---- verdict -------------------------------------------------------
    print()
    print("=" * 60)
    if total_lost == 0 and duplicates == 0 and resets == 0:
        print("VERDICT: CLEAN - seq increments by 1 every line, no loss.")
    else:
        print("VERDICT: PROBLEM DETECTED")
        if total_lost:
            print(f"  - {total_lost} packet(s) lost across {len(gaps)} gap(s).")
        if duplicates:
            print(f"  - {duplicates} duplicate/stale seq read(s).")
        if resets:
            print(f"  - {resets} seq reset(s) (counter went backwards).")
    print("=" * 60)


def main():
    p = argparse.ArgumentParser(
        description="Analyze seq integrity of a com_log file.")
    p.add_argument("--file", default="log.txt", help="input log file")
    args = p.parse_args()
    analyze(args.file)


if __name__ == "__main__":
    main()
