"""
Why the classic HJ filter (filter mode ``hj``) collides: for every collision or near collision (``filter_error``:
the robot left the free region shrunk by the footprint; only with the earlier HJ filter, which built the region) of MPPI + hj in the worlds of filter_batch.py, find the
period in which the robot entered the true BRT (after it, no control avoids the obstacles for the horizon T), and
classify the failure from that state:

  leaking corner  -- U = min_i V_i > 0 there and every face alone can still be escaped (its rectangle only), but
                     not all of them together: the union of the primitive values leaks;
  single face     -- U > 0 but one face alone can no longer be escaped: V0 underestimates that face's BRT;
  U <= 0 already  -- the filter knew, its evasion came too late or did not work.

U is the value the filter sees: with --corners, the union lowered at the corrected corners (docs/13).

"Can be escaped" is checked with a rich set of rollouts (hold w1 for k steps, then w2; w in {-1, -0.5, 0, 0.5, 1}
and {-1, 0, 1}, switches every 2 sub-steps), exact arcs, margin > 0 at every sub-step over T = 2 (model units).
It is a lower bound: an escape the set misses counts as no escape.  If the true world can still be escaped at the
end (a near collision with the mapped obstacle), the scene's modelled obstacle is used instead of the world.

    cd nav_core
    PYTHONPATH=. python experiments/hj_failures.py                 # hj and occluded worlds, MPPI
    PYTHONPATH=. python experiments/hj_failures.py --worlds hj --nominal pursuit --verbose
    PYTHONPATH=. python experiments/hj_failures.py --margin 0.02 --robust
"""

from __future__ import annotations

import argparse

import numpy as np

from filter_batch import cases, worlds
from nav_core.mapping import OccupancyGrid
from nav_core.nominal import make_nominal
from nav_core.robots import load_profile
from nav_core.safety import HJFilter, load_v0
from nav_core.safety.witness import arc_step
from nav_core.sim import run_episode

T, DT = 2.0, 0.05
K = int(round(T / DT))


def escape_controls() -> np.ndarray:
    rows = [np.r_[np.full(k1, w1), np.full(K - k1, w2)]
            for w1 in (-1.0, -0.5, 0.0, 0.5, 1.0) for k1 in range(0, K + 1, 2) for w2 in (-1.0, 0.0, 1.0)]
    return np.unique(np.array(rows), axis=0)


W = escape_controls()


def escapes(xm, margin) -> bool:
    """Some rollout from xm (model units) keeps margin(xy) > 0 over T."""
    z = np.repeat(np.asarray(xm, float)[None], len(W), 0)
    ok = margin(z[:, :2]) > 0
    for k in range(K):
        arc_step(z, W[:, k], DT)
        ok &= margin(z[:, :2]) > 0
    return bool(ok.any())


def rectangle_margin(frame):
    c, hl, hw, b = frame
    cb, sb = np.cos(b), np.sin(b)

    def margin(xy):
        dx, dy = xy[:, 0] - c[0], xy[:, 1] - c[1]
        return np.maximum(np.abs(cb * dx + sb * dy) - hl, np.abs(-sb * dx + cb * dy) - hw)
    return margin


class LoggingHJFilter(HJFilter):
    """HJFilter that keeps (state, U, nearest face, scene) of every period."""

    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.log = []

    def __call__(self, x, w_nom, grid):
        out = super().__call__(x, w_nom, grid)
        xm = self.to_model(x)
        q = self.v0.query(xm[None], self.model.faces)
        # U as the filter sees it (with the corrected corners if any); the nearest face for the classification
        self.log.append((xm, float(self.model.value(xm[None])[0]), int(q.face[0]), self.model))
        return out


def classify(gate, true_margin):
    flags = [escapes(xm, true_margin) for xm, *_ in gate.log]
    which = "world"
    if flags[-1]:                                     # near collision with the mapped obstacle: use the scene
        flags, which = [escapes(xm, scene.margin) for xm, _, _, scene in gate.log], "scene"
    entry = max([k for k, f in enumerate(flags) if f], default=-1) + 1
    xm, u, face, scene = gate.log[entry]
    alone = [escapes(xm, rectangle_margin(frame)) for frame in scene.frames]
    nearest = {gate.log[k][2] for k in range(entry, len(gate.log))}
    kind = ("U <= 0 already" if u <= 0 else "single face (V0 too small)" if not all(alone)
            else "leaking corner")
    return {"entry_periods_before_end": len(gate.log) - entry, "U_at_entry": u, "faces_escapable": sum(alone),
            "faces": len(alone), "nearest_faces_after_entry": len(nearest), "checked_against": which, "kind": kind}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worlds", nargs="+", default=["hj", "occluded"], choices=["hj", "pillar", "occluded"])
    ap.add_argument("--nominal", default="mppi", choices=["mppi", "pursuit"])
    ap.add_argument("--margin", type=float, default=0.0, help="HJ filter margin (m)")
    ap.add_argument("--robust", action="store_true", help="HJ filter: add the clearance and the sampling gap")
    ap.add_argument("--corners", action="store_true", help="correct the leaking corners (docs/13)")
    ap.add_argument("--corner-theta", type=int, default=None)
    ap.add_argument("--verbose", action="store_true", help="also print U and the escapes of the last 8 periods")
    args = ap.parse_args()

    profile = load_profile("burger")
    v0 = load_v0(profile.base_value)
    units = profile.units
    counts = {}
    for wname in args.worlds:
        world = worlds(profile)[wname]

        def true_margin(xy, w=world["w"]):
            return (w.distance(units.pos_to_phys(xy)) - profile.footprint_radius) / units.length

        for i, (x0, goal) in enumerate(cases(world, profile)):
            gate = LoggingHJFilter(profile, v0, margin=args.margin, robust=args.robust, corners=args.corners,
                                  corner_theta=args.corner_theta)
            grid = None if world["grid"] is None else OccupancyGrid(*world["grid"], profile.map.resolution)
            nominal = make_nominal(profile, args.nominal, **({"seed": i} if args.nominal == "mppi" else {}))
            ep = run_episode(profile, world["w"], x0, goal, nominal, safety=gate, t_max=100, seed=i, grid=grid)
            if ep.status in ("reached", "timeout"):
                continue
            r = classify(gate, true_margin)
            counts[r["kind"]] = counts.get(r["kind"], 0) + 1
            print(f"[{wname} case {i:2d}] {ep.status:12s}: enters the {r['checked_against']}'s BRT "
                  f"{r['entry_periods_before_end']} periods before the end, U there {r['U_at_entry']:+.3f}, faces "
                  f"alone escapable {r['faces_escapable']}/{r['faces']}, nearest faces after it "
                  f"{r['nearest_faces_after_entry']} -> {r['kind']}", flush=True)
            if args.verbose:
                for xm, u, face, scene in gate.log[-8:]:
                    print(f"    U {u:+6.3f}  nearest face {face:2d}  escape: world {escapes(xm, true_margin)}, "
                          f"nearest face alone {escapes(xm, rectangle_margin(scene.frames[face]))}")
    print("summary:", counts)


if __name__ == "__main__":
    main()
