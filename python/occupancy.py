"""Predicted occupancy grid -- a probabilistic world-model layer.

Note: the production implementation is the C++ version exposed as ``occupancy_cpp``
(built via pybind11). This pure-Python module is a legacy/reference implementation
retained for the older debug/demo scripts (e.g. debug_avoidance.py).


Given the tracker's predicted agent trajectories, build a 2-D grid over the road
ahead whose cells hold the probability of being occupied at a chosen future time.
Each agent is splatted as a vehicle-footprint Gaussian whose spread grows with the
prediction horizon (uncertainty inflation), combined across agents by a
probabilistic OR. Downstream, the planner can integrate occupancy along a
candidate trajectory as a soft cost, and the guardrail can veto a path that passes
through high-probability occupied cells -- a continuous alternative to discrete
collision checks.
"""
import numpy as np


class OccupancyGrid:
    def __init__(self, x_min, x_range=80.0, y_min=-2.0, y_max=5.5, res=0.5,
                 sigma0=0.6, growth=0.06, car_l=2.2, car_w=0.9):
        self.res = res
        self.xs = np.arange(x_min, x_min + x_range, res)
        self.ys = np.arange(y_min, y_max, res)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)
        self.x0, self.y0 = x_min, y_min
        self.sigma0, self.growth = sigma0, growth
        self.car_l, self.car_w = car_l, car_w

    def predict(self, tracks_pred, horizons):
        """tracks_pred: list of predicted (x,y) lists; horizons: list of step indices.
        Returns one occupancy grid (ny, nx) per horizon."""
        grids = []
        for h in horizons:
            g = np.zeros_like(self.X)
            for pred in tracks_pred:
                if not pred:
                    continue
                j = min(h, len(pred) - 1)
                px, py = pred[j]
                s = self.sigma0 + self.growth * h          # uncertainty grows with horizon
                sl, sw = self.car_l + s, self.car_w + s
                p = np.exp(-0.5 * ((self.X - px) ** 2 / sl ** 2 + (self.Y - py) ** 2 / sw ** 2))
                g = 1 - (1 - g) * (1 - p)                  # probabilistic OR
            grids.append(g)
        return grids

    def sample(self, grid, x, y):
        """Occupancy probability at world point (x, y) (nearest cell)."""
        i = int(round((y - self.y0) / self.res)); j = int(round((x - self.x0) / self.res))
        if 0 <= i < grid.shape[0] and 0 <= j < grid.shape[1]:
            return float(grid[i, j])
        return 0.0

    def path_risk(self, grid, xs, ys):
        """Max occupancy probability encountered along a path."""
        return max((self.sample(grid, x, y) for x, y in zip(xs, ys)), default=0.0)


if __name__ == "__main__":
    import os, sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build"))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build", "Release"))
    import world as W, sensors as S, perception as P, world_cpp as WC

    np.random.seed(0); S.rng = np.random.default_rng(0)
    wd = W.scenario_mixed(); suite = S.SensorSuite()
    mot = WC.MultiObjectTracker(); mot.dt = 0.1; mot.min_hits = 2
    ego = [0.0, 0.0, 0.0, 12.0]
    for _ in range(8):                       # warm up the tracker
        dets = []
        for f in P.perceive(suite, ego, wd.true_objects()):
            hv = f.v_prior is not None; vx, vy = (f.v_prior if hv else (0.0, 0.0))
            dets.append((f.z[0], f.z[1], f.R[0, 0], f.R[0, 1], f.R[1, 0], f.R[1, 1],
                         int(hv), float(vx), float(vy)))
        tracks = mot.step(dets); wd.step(0.1)
    preds = [mot.predict(t[0], 40) for t in tracks if t[6]]
    preds = [p for p in preds if p]

    grid = OccupancyGrid(x_min=ego[0], x_range=100.0)
    horizons = [5, 15, 30]
    grids = grid.predict(preds, horizons)
    print(f"confirmed tracks: {len(preds)}")
    for h, g in zip(horizons, grids):
        print(f"  horizon {h*0.1:.1f}s: peak occupancy {g.max():.2f}, "
              f"cells>0.5: {(g > 0.5).sum()}")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(len(horizons), 1, figsize=(11, 6), sharex=True)
        for a, h, g in zip(ax, horizons, grids):
            im = a.imshow(g, origin="lower", aspect="auto", cmap="inferno",
                          extent=[grid.xs[0], grid.xs[-1], grid.ys[0], grid.ys[-1]], vmin=0, vmax=1)
            for pred in preds:
                j = min(h, len(pred) - 1); a.plot(pred[j][0], pred[j][1], "co", ms=5)
            a.set_ylabel("y [m]"); a.set_title(f"predicted occupancy at +{h*0.1:.1f}s")
        ax[-1].set_xlabel("x [m]")
        fig.colorbar(im, ax=ax, label="P(occupied)", fraction=0.025)
        out = os.path.join(os.path.dirname(__file__), "..", "assets", "occupancy_pred.png")
        plt.savefig(out, dpi=95, bbox_inches="tight")
        print(f"saved {out}")
    except Exception as e:
        print("plot skipped:", e)
