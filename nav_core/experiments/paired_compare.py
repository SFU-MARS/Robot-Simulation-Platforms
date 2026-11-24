"""
Paired comparison of batches that differ in one setting only, episode by episode (filter_batch.py --episodes-out
writes the episodes: the same world and episode index means the same start, goal and seeds). Made for "the same
backup certificate, with and without the local update (corner correction)":

    cd nav_core
    PYTHONPATH=. python experiments/paired_compare.py outputs/paired/ep_*.csv --base off

Labels are <robot>_<filter>_<mode> (filter_batch.py --label); every mode is compared with ``--base`` for the same
robot, filter and world (each gap width of the gap worlds on its own). Printed per comparison (Markdown):

  reached     base -> mode; discordant pairs (reached only with base / only with mode) and the two-sided exact
              sign test on them
  collisions  base -> mode (the certificate is the same, so both should be 0)
  rejections  backup rejections per control period (the backup filter's interventions; hj_backup: the periods in
              which the backup filter replaced the HJ filter's rates), pooled; paired Wilcoxon signed-rank test on
              the per-episode rates
  search      witness search time per control period (ms) and the share of witnesses that were the HJ candidate
  time, path  on the episodes both reached: median paired difference of the time to the goal (s) and of the path
              length over the straight distance, Wilcoxon test
  deadline    95th percentile and maximum of the control step time (ms, without corner solves, which run on a
              worker online), periods over the control period, corner solves per control period
"""

from __future__ import annotations

import argparse
import collections
import csv
import glob

import numpy as np
from scipy.stats import binomtest, wilcoxon


def load(paths):
    rows = []
    for pattern in paths:
        for path in sorted(glob.glob(pattern)):
            rows += list(csv.DictReader(open(path)))
    for r in rows:
        robot, rest = r["label"].split("_", 1)
        filt, mode = rest.rsplit("_", 1)
        r["robot"], r["filter"], r["mode"] = robot, filt, mode
    return rows


def f(x) -> float:
    return float(x) if x not in ("", None) else 0.0


def wilcox(a, b) -> str:
    d = np.asarray(b) - np.asarray(a)
    if len(d) < 6 or np.allclose(d, 0):
        return "–"
    return f"{wilcoxon(a, b).pvalue:.3f}"


def compare(base, other):
    """base, other: {episode: row} of one robot, filter and world."""
    keys = sorted(set(base) & set(other), key=int)
    b, o = [base[k] for k in keys], [other[k] for k in keys]
    reached_b = np.array([r["status"] == "reached" for r in b])
    reached_o = np.array([r["status"] == "reached" for r in o])
    only_b, only_o = int(np.sum(reached_b & ~reached_o)), int(np.sum(~reached_b & reached_o))
    sign_p = binomtest(only_o, only_b + only_o).pvalue if only_b + only_o else 1.0
    steps = lambda rows: sum(f(r["steps"]) for r in rows)
    rate = lambda r: f(r["backup_rejections"]) / max(f(r["steps"]), 1)
    both = reached_b & reached_o
    tb = [f(r["t"]) for r, m in zip(b, both) if m]
    to = [f(r["t"]) for r, m in zip(o, both) if m]
    ratio = lambda r: f(r["path_m"]) / max(f(r["straight_m"]), 1e-9)
    pb = [ratio(r) for r, m in zip(b, both) if m]
    po = [ratio(r) for r, m in zip(o, both) if m]
    found = lambda rows: sum(f(r["searches_found"]) for r in rows)
    return {
        "n": len(keys),
        "reached": f"{int(reached_b.sum())} → {int(reached_o.sum())}",
        "discordant": f"{only_b} / {only_o} (p {sign_p:.2f})",
        "collisions": f"{sum(r['status'] == 'collision' for r in b)} → {sum(r['status'] == 'collision' for r in o)}",
        "rejections": f"{sum(f(r['backup_rejections']) for r in b) / max(steps(b), 1):.3f} → "
                      f"{sum(f(r['backup_rejections']) for r in o) / max(steps(o), 1):.3f} "
                      f"(p {wilcox([rate(r) for r in b], [rate(r) for r in o])})",
        "search": f"{sum(f(r['search_ms']) for r in b) / max(steps(b), 1):.1f} → "
                  f"{sum(f(r['search_ms']) for r in o) / max(steps(o), 1):.1f} ms; HJ witness "
                  f"{sum(f(r['searches_hj']) for r in b) / max(found(b), 1):.2f} → "
                  f"{sum(f(r['searches_hj']) for r in o) / max(found(o), 1):.2f}",
        "time": (f"{np.median(np.array(to) - np.array(tb)):+.1f} s (p {wilcox(tb, to)}, {len(tb)} pairs)"
                 if tb else "–"),
        "path": f"{np.median(np.array(po) - np.array(pb)):+.3f} (p {wilcox(pb, po)})" if pb else "–",
        "deadline": f"p95 {np.percentile([f(r['step_ms_p95']) for r in o], 95):.0f} ms, max "
                    f"{max(f(r['step_ms_max']) for r in o):.0f} ms, {int(sum(f(r['deadline_misses']) for r in o))} "
                    f"over the period; {sum(f(r['corner_solves']) for r in o) / max(steps(o), 1):.2f} solves / period",
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="episode CSVs (filter_batch.py --episodes-out), globs allowed")
    ap.add_argument("--base", default="off", help="mode to compare the others with")
    args = ap.parse_args()
    rows = load(args.paths)
    groups = collections.defaultdict(dict)                # (robot, filter, world, mode) -> {episode: row}
    for r in rows:
        groups[(r["robot"], r["filter"], r["world"], r["mode"])][r["episode"]] = r
    cols = ["n", "reached", "discordant", "collisions", "rejections", "search", "time", "path", "deadline"]
    print("| robot | filter | world | mode | " + " | ".join(cols) + " |")
    print("| " + " | ".join(["---"] * (4 + len(cols))) + " |")
    for (robot, filt, world, mode) in sorted(groups):
        if mode == args.base or (robot, filt, world, args.base) not in groups:
            continue
        res = compare(groups[(robot, filt, world, args.base)], groups[(robot, filt, world, mode)])
        print(f"| {robot} | {filt} | {world} | {args.base} → {mode} | " + " | ".join(str(res[c]) for c in cols) + " |")


if __name__ == "__main__":
    main()
