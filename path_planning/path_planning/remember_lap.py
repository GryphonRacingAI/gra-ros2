"""Remembered lap (opt-in, closed circuits only; no ROS imports).

The line the car drove on lap 1 is stored in the map frame; from lap 2 on it is the path, as long as the
car is on it and it stays clear of every cone the car can see now. Otherwise the caller plans live for
that update (track changed, a cone was moved, or the car is not where it thinks it is).
"""
import math

import numpy as np

from path_planning.geometry import to_local


class RememberedLap:
    def __init__(self, logger, min_clear=0.75, max_offset=1.5):
        self.logger = logger
        self.min_clear = min_clear      # m, path centre to a cone
        self.max_offset = max_offset    # m, car to the stored line
        self.trace = []                 # map-frame points driven so far (lap 1)
        self.loop = None                # (N, 2) closed loop in the map frame, 0.5 m spacing
        self.idx = 0
        self.mode = None                # 'remembered' / 'live', for the log

    def record(self, rx, ry, laps):
        """Lap 1: store where the car drives (map frame, a point every 0.5 m). When the first lap is
        counted, close the loop where the trace comes back to its own beginning."""
        if self.loop is not None or self.trace is None:
            return
        if not self.trace or math.hypot(rx - self.trace[-1][0], ry - self.trace[-1][1]) >= 0.5:
            self.trace.append((rx, ry))
        if laps < 1 or len(self.trace) < 60:
            return
        pts = np.array(self.trace)
        head = pts[:max(20, len(pts) // 5)]                       # where the lap began
        d = np.hypot(*(head - pts[-1]).T); j = int(np.argmin(d))
        if d[j] > 2.0:
            if len(pts) > 4000:                                    # never came back: give up
                self.logger.warn("[PATHFINDER] remember_lap: the lap did not close, staying with live planning")
                self.trace = None
            return
        loop = pts[j:]
        # light smoothing of the driven line (closed loop, +-1 m), it is followed as it is otherwise
        k = 2; pad = np.vstack([loop[-k:], loop, loop[:k]])
        self.loop = np.array([pad[i:i + 2 * k + 1].mean(axis=0) for i in range(len(loop))])
        self.idx = 0
        self.logger.info(f"[PATHFINDER] remember_lap: lap 1 stored, {len(loop)} points, "
                               f"{np.hypot(*np.diff(loop, axis=0).T).sum():.0f} m, closes within {d[j]:.2f} m")

    def path(self, rx, ry, r_yaw, cones_local, length=20.0):
        """The stored lap ahead of the car in base_link, or None if it cannot be trusted right now.
        cones_local: arrays of (N, 2) cones in base_link that the car can see now, any colour."""
        if self.loop is None:
            return None
        n = len(self.loop)
        win = (self.idx + np.arange(-6, 40)) % n               # only look around the last position
        d = np.hypot(self.loop[win, 0] - rx, self.loop[win, 1] - ry)
        if d.min() > self.max_offset:                    # lost it: search the whole loop once
            d_all = np.hypot(self.loop[:, 0] - rx, self.loop[:, 1] - ry)
            if d_all.min() > self.max_offset:
                return self._set_mode('live', f"car is {d_all.min():.1f} m from the stored lap")
            self.idx = int(np.argmin(d_all))
        else:
            self.idx = int(win[int(np.argmin(d))])
        ahead = self.loop[(self.idx + 2 + np.arange(int(length / 0.5))) % n]
        local = to_local(ahead, rx, ry, r_yaw)
        gap = np.where(np.hypot(*np.diff(local, axis=0).T) > 2.0)[0]
        if len(gap):
            local = local[:gap[0] + 1]                           # never bridge a hole in the stored line
        if len(local) < 8 or local[0][0] < -0.5:
            return self._set_mode('live', "stored lap is not ahead of the car")
        cones = [np.asarray(c, dtype=float).reshape(-1, 2) for c in cones_local if len(c)]
        if cones:
            c = np.vstack(cones); c = c[(c[:, 0] > -1.0) & (np.hypot(c[:, 0], c[:, 1]) < length + 2.0)]
            if len(c):
                clear = np.hypot(local[:, None, 0] - c[None, :, 0], local[:, None, 1] - c[None, :, 1]).min()
                if clear < self.min_clear:
                    return self._set_mode('live', f"a cone is {clear:.2f} m from the stored lap")
        self._set_mode('remembered', "following the stored lap")
        return np.vstack([[0.0, 0.0], local])

    def _set_mode(self, mode, why):
        if mode != self.mode:
            self.mode = mode
            self.logger.info(f"[PATHFINDER] remember_lap -> {mode}: {why}")
        return None
