#!/usr/bin/env python3
"""Keep only the trials of a terrain plan (E3/E4/E5) that ran before the world state became
uncertain, so the run can continue with --resume (run_all.sh does that).

    python3 scripts/trim_contaminated.py results/e3 results/e4

Why (night of 25-26 Sep 2026): gz service replies were lost ("Host unreachable"), the
runner took executed create/remove requests for failures, and later trials walked on
leftover / overlapping terrain pieces. The first "SKIP - could not create" or
"WARNING - terrain piece not removed" line with "timed out" in run_e2_log.txt marks the
point after which no trial can be trusted; that trial itself (removal happens after its
walk) is kept.

Per directory: e2_trials.csv -> e2_trials.run1_full.csv (untouched copy) and
run_e2_log.txt -> run_e2_log.run1.txt, then e2_trials.csv is rewritten with the clean
prefix. Refuses to run twice (the copies already exist).
"""
import csv
import os
import re
import shutil
import sys

LINE = re.compile(r"^(\S+): (OK  |FAIL|SKIP|WARNING)")


def trim(directory: str) -> int:
    csv_path = os.path.join(directory, "e2_trials.csv")
    log_path = os.path.join(directory, "run_e2_log.txt")
    full_path = os.path.join(directory, "e2_trials.run1_full.csv")
    if os.path.exists(full_path):
        print(f"{directory}: already trimmed ({full_path} exists) - nothing done")
        return 0
    if not (os.path.exists(csv_path) and os.path.exists(log_path)):
        print(f"{directory}: no e2_trials.csv / run_e2_log.txt - nothing done")
        return 0
    keep, anomaly = [], None
    with open(log_path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = LINE.match(line)
            if not match:
                continue
            trial, kind = match.groups()
            if kind in ("SKIP", "WARNING") and "timed out" in line:
                anomaly = line.strip()
                break
            if kind in ("OK  ", "FAIL"):
                keep.append(trial)
    with open(csv_path, encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields, rows = reader.fieldnames, list(reader)
    if anomaly is None:
        print(f"{directory}: no lost gz reply in the log - all {len(rows)} trial(s) kept")
        return 0
    if [r["trial"] for r in rows[:len(keep)]] != keep:
        print(f"{directory}: csv order does not match the log - nothing done", file=sys.stderr)
        return 1
    shutil.copy2(csv_path, full_path)
    shutil.copy2(log_path, os.path.join(directory, "run_e2_log.run1.txt"))
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows[:len(keep)])
    with open(log_path, "a", encoding="utf-8") as handle:
        handle.write(f"\n=== trim_contaminated: kept {len(keep)} of {len(rows)} trial(s) "
                     f"(up to {keep[-1] if keep else '-'}); first lost reply: {anomaly} ===\n")
    print(f"{directory}: kept {len(keep)} of {len(rows)} trial(s) (up to {keep[-1] if keep else '-'}); "
          f"first lost reply: {anomaly[:100]}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(max(trim(d) for d in sys.argv[1:]))
