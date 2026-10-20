"""Dirt grid on every tread; cells are cleaned when the lowered mop/vacuum head passes over them."""
import numpy as np
import pybullet as p

import config as C

DIRT_RGBA = [0.45, 0.30, 0.15, 1]
CLEAN_RGBA = [0.45, 0.30, 0.15, 0]


class DirtMap:
    def __init__(self, stairs, cell=0.04, visual=True, landing_len=0.5, seed=0):
        self.cell = cell
        self.visual = visual
        self.treads = {}     # tread index -> dict(xs, ys, z, clean bool grid, ids)
        rng = np.random.default_rng(seed)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[cell * 0.35, cell * 0.35, 0.001],
                                  rgbaColor=DIRT_RGBA) if visual else None
        for i in range(1, stairs.n_steps + 1):
            x0, x1, z = stairs.tread_bounds(i)
            if i == stairs.n_steps:
                x1 = x0 + landing_len
            xs = np.arange(x0 + cell / 2, x1, cell)
            ys = np.arange(-stairs.width / 2 + cell / 2, stairs.width / 2, cell)
            ids = None
            if visual:
                pos = [[x + rng.uniform(-0.01, 0.01), y + rng.uniform(-0.01, 0.01), z + 0.0015]
                       for x in xs for y in ys]
                ids = np.array(p.createMultiBody(baseVisualShapeIndex=vis, batchPositions=pos)).reshape(len(xs), len(ys))
            self.treads[i] = dict(xs=xs, ys=ys, z=z, clean=np.zeros((len(xs), len(ys)), bool), ids=ids)

    def clean_at(self, x, y, z, half_x, half_y):
        """Mark cells under a rectangular head footprint (world coords)."""
        for t in self.treads.values():
            if abs(t["z"] - z) > 0.04:
                continue
            ix = np.where(np.abs(t["xs"] - x) <= half_x)[0]
            iy = np.where(np.abs(t["ys"] - y) <= half_y)[0]
            if len(ix) == 0 or len(iy) == 0:
                continue
            sub = t["clean"][np.ix_(ix, iy)]
            newly = np.argwhere(~sub)
            if len(newly) == 0:
                continue
            t["clean"][np.ix_(ix, iy)] = True
            if self.visual:
                for a, b in newly:
                    p.changeVisualShape(t["ids"][ix[a], iy[b]], -1, rgbaColor=CLEAN_RGBA)

    def coverage(self, i=None):
        if i is not None:
            return float(self.treads[i]["clean"].mean())
        tot = sum(t["clean"].size for t in self.treads.values())
        return float(sum(t["clean"].sum() for t in self.treads.values()) / tot)

    def reachable_coverage(self, i, span):
        """Coverage of the band between the two side units (what the central
        cleaning module can physically reach)."""
        t = self.treads[i]
        inner = span / 2 - C.UNIT_WIDTH / 2
        mask = np.abs(t["ys"]) <= inner
        return float(t["clean"][:, mask].mean())
