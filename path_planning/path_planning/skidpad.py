"""Skidpad planner: the figure of eight from the rules, anchored on the mapped cones (no ROS imports)."""
import math

import numpy as np


class SkidpadPlanner:
    """Skidpad: entry, right circle x2, left circle x2, exit (FS rules), anchored on the cones.

    Layout from the rules: circle centres 9.125 m either side of the timing line centre, inner
    cone radius 7.625 m, outer 10.625 m. The timing line centre is first the mean of the large
    orange cones in the SLAM map, then refined by fitting all mapped blue/yellow cones to the two
    circles. The driving direction is the car's initial heading (map +x): the car starts on the
    entry lane facing the timing line. Progress along the path only moves forward (searched in a
    small window), so the crossing in the middle of the figure 8 can't make it skip or repeat.
    """
    R_CENTRE, R_INNER, R_OUTER = 9.125, 7.625, 10.625

    def __init__(self, logger, start_distance=15.0, spacing=0.5, window=20.0, exit_length=25.0):
        self.logger = logger
        self.spacing = spacing
        self.window = int(window / spacing)
        self.centre = np.array([start_distance, 0.0])  # timing line centre (map), until cones are mapped
        self.anchored = False
        self.frozen = False
        self.idx = 0
        # path in the skidpad frame: origin = timing line centre, +x = driving direction, +y = left
        pts = [np.array([x, 0.0]) for x in np.arange(-start_distance - 5.0, 0.0, spacing)]
        n = int(2 * math.pi * self.R_CENTRE / spacing)
        for lap in range(2):  # right circle (clockwise), centre (0, -R)
            for k in range(n):
                phi = math.pi / 2 - 2 * math.pi * k / n
                pts.append(np.array([self.R_CENTRE * math.cos(phi), -self.R_CENTRE + self.R_CENTRE * math.sin(phi)]))
        for lap in range(2):  # left circle (counter-clockwise), centre (0, +R)
            for k in range(n):
                phi = -math.pi / 2 + 2 * math.pi * k / n
                pts.append(np.array([self.R_CENTRE * math.cos(phi), self.R_CENTRE + self.R_CENTRE * math.sin(phi)]))
        pts += [np.array([x, 0.0]) for x in np.arange(0.0, exit_length, spacing)]
        self.path = np.array(pts)
        self.entry_end = int((start_distance + 5.0) / spacing)

    def update_anchor(self, orange, blue, yellow):
        """Locate the timing line centre from the mapped cones (map frame)."""
        cones = np.array(list(blue) + list(yellow)) if len(blue) + len(yellow) else np.empty((0, 2))
        if self.frozen or len(cones) < 10:
            return
        # Start from the large orange cones if there are some, but let the circles decide: the
        # orange mean alone was 6 m off (small entry-lane cones taken for large ones), so search
        # +-8 m along the driving direction and +-2 m across.
        guess = np.mean(orange, axis=0) if len(orange) >= 2 else self.centre.copy()
        best = guess
        if len(cones) >= 10:
            best_err = float('inf')
            for dx in np.arange(-8.0, 8.01, 0.25):
                for dy in np.arange(-2.0, 2.01, 0.25):
                    c = guess + [dx, dy]
                    d = np.minimum(np.linalg.norm(cones - (c + [0, -self.R_CENTRE]), axis=1),
                                   np.linalg.norm(cones - (c + [0, self.R_CENTRE]), axis=1))
                    r = np.minimum(np.abs(d - self.R_INNER), np.abs(d - self.R_OUTER))
                    err = np.sum(np.minimum(r, 1.0) ** 2)  # robust to stray cones
                    if err < best_err:
                        best, best_err = c, err
        self.centre = best if not self.anchored else 0.8 * self.centre + 0.2 * best
        if not self.anchored:
            self.logger.info(f"[SKIDPAD] timing line found at map ({best[0]:.2f}, {best[1]:.2f})")
        self.anchored = True

    def plan(self, car_map_xy):
        """Path window ahead of the car in the map frame, and whether the skidpad is done."""
        local = self.path + self.centre
        lo, hi = max(0, self.idx - 4), min(len(local), self.idx + self.window)
        self.idx = lo + int(np.argmin(np.linalg.norm(local[lo:hi] - car_map_xy, axis=1)))
        if self.idx > self.entry_end and self.anchored and not self.frozen:
            self.frozen = True  # don't move the circles under the car
            self.logger.info("[SKIDPAD] entered the circles, layout frozen")
        done = self.idx >= len(local) - 4
        return local[self.idx:self.idx + self.window], done
