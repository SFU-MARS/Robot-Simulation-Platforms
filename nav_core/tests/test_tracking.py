import json

import numpy as np

from nav_core.mapping import ObstacleTracker, OccupancyGrid, extract_obstacles

RES = 0.02


def grid_with(*boxes):
    """Known-free 2 x 2 m map with occupied boxes (xmin, xmax, ymin, ymax)."""
    g = OccupancyGrid((-1, -1), (2, 2), RES, model="logodds")
    g.logodds[:] = -2.0
    for x0, x1, y0, y1 in boxes:
        g.logodds[int(round((y0 + 1) / RES)):int(round((y1 + 1) / RES)),
                  int(round((x0 + 1) / RES)):int(round((x1 + 1) / RES))] = 2.0
    return g


def step(tr, g, t):
    om, ev = tr.update(extract_obstacles(g), t, RES)
    return om, ev


def test_new_then_updated_as_more_is_seen_and_quiet_otherwise():
    tr = ObstacleTracker()
    _, ev = step(tr, grid_with((0.0, 0.2, 0.0, 0.06)), 0.0)
    assert [(e.kind, e.obstacle) for e in ev] == [("new", 1)]
    _, ev = step(tr, grid_with((0.0, 0.2, 0.0, 0.06)), 0.2)
    assert ev == []                                            # nothing changed: no event
    _, ev = step(tr, grid_with((0.0, 0.5, 0.0, 0.06)), 0.4)    # the wall turns out longer
    assert [(e.kind, e.obstacle) for e in ev] == [("updated", 1)]
    assert max(f.length for f in ev[0].faces) > 0.45


def test_ids_are_stable_when_raw_labels_change():
    tr = ObstacleTracker()
    a = (0.3, 0.5, 0.3, 0.5)
    om, _ = step(tr, grid_with(a), 0.0)
    ida = int(om.labels[int((0.4 + 1) / RES), int((0.4 + 1) / RES)])
    # a second box below the first gets raw label 1 from extract_obstacles (raster order)
    om, ev = step(tr, grid_with(a, (-0.5, -0.3, -0.5, -0.3)), 0.2)
    assert int(om.labels[int((0.4 + 1) / RES), int((0.4 + 1) / RES)]) == ida
    assert [(e.kind, e.obstacle) for e in ev] == [("new", ida + 1)]
    assert {f.obstacle for f in om.faces} == {ida, ida + 1}


def test_merge_keeps_the_oldest_id():
    tr = ObstacleTracker()
    step(tr, grid_with((-0.5, -0.1, 0.0, 0.06), (0.1, 0.5, 0.0, 0.06)), 0.0)
    om, ev = step(tr, grid_with((-0.5, 0.5, 0.0, 0.06)), 0.2)   # the gap between them gets filled
    assert om.n == 1
    assert [(e.kind, e.obstacle, e.merged_from) for e in ev] == [("merged", 1, (2,))]


def test_small_fragments_are_not_reported_and_removal_is():
    tr = ObstacleTracker()
    _, ev = step(tr, grid_with((0.0, 0.04, 0.0, 0.02)), 0.0)    # 2 cells: a stray hit
    assert ev == []
    step(tr, grid_with((0.0, 0.2, 0.0, 0.06)), 0.2)
    _, ev = step(tr, grid_with(), 0.4)
    assert [(e.kind, e.obstacle) for e in ev] == [("removed", 1)]


def test_events_serialize_to_json():
    tr = ObstacleTracker()
    _, ev = step(tr, grid_with((0.0, 0.2, 0.0, 0.2)), 0.0)
    d = json.loads(json.dumps(ev[0].to_dict()))
    assert d["kind"] == "new" and len(d["faces"]) == 4 and sum(c["convex"] for c in d["corners"]) == 4


def test_unreported_fragment_merging_into_a_reported_obstacle():
    tr = ObstacleTracker()
    step(tr, grid_with((0.0, 0.04, 0.0, 0.02)), 0.0)                     # id 1: 2 cells, not reported
    _, ev = step(tr, grid_with((0.0, 0.04, 0.0, 0.02), (0.2, 0.6, 0.0, 0.06)), 0.2)
    assert [(e.kind, e.obstacle) for e in ev] == [("new", 2)]
    _, ev = step(tr, grid_with((0.0, 0.6, 0.0, 0.06)), 0.4)              # the wall reaches the fragment
    assert [(e.kind, e.obstacle, e.merged_from) for e in ev] == [("merged", 1, (2,))]
