"""Centreline planner: cones in the car frame -> waypoints in the car frame (no ROS imports)."""
import math

import numpy as np


class CentrelineAlgorithm:
    """
    Centreline Pathfinder.
    geometric midpoint with path smoothing.
    """
    def __init__(self, logger, lookahead_distance: float = 20.0):
        self.logger = logger
        self.lookahead_distance = lookahead_distance
        
        self.min_x_lookahead = 0.5
        self.max_gate_distance = 8.0
        self.smoothing_window = 3
        self.cone_vertical_tolerance = 1.5 
        self.default_track_width = 3.0   # m, used until two real gates are in view
        self.lone_cone_factor = 1.35     # nearest other-colour cone further than this x track width: partner missing
        self.lone_cone_bias = 0.45       # m a one-boundary waypoint is moved from the centre towards its (mapped) boundary
        self.lone_partner_max = 14.0     # m, a far "partner" beyond this does not even tell which side the track is on
        self.lone_cone_range = 10.0      # m, cones without a partner in max_gate_distance are only used this close to the car
        self.min_cone_clearance = 0.3    # x track width: a one-boundary waypoint closer than this to a mapped cone is dropped
        self.chain_max_step = 8.0        # m between consecutive waypoints
        self.chain_max_turn = math.radians(75.0)   # largest heading change between consecutive waypoints
        
        # Last 2 Previously published /path values
        self.path_buffer : np.ndarray

    def calculate_path(self, yellow_cones, blue_cones):
        """
        Calculate centreline using geometric midpoint + smoothing
        Input: yellow_cones, blue_cones as [[x, y], ...] in base_link (car-local)
        Output: smoothed waypoints [[x, y], ...] in base_link or None
        
        Algorithm:
        1. Filter cones ahead of car
        2. Sort by forward distance
        3. Greedily pair nearest Yellow-Blue cones
        4. Compute midpoints as centreline candidates
        5. Smooth waypoints *need to check if needed*
        """
        
        if len(yellow_cones) == 0 and len(blue_cones) == 0:
            return None
        
        y_np = np.array(yellow_cones) if len(yellow_cones) > 0 else np.empty((0, 2))
        b_np = np.array(blue_cones) if len(blue_cones) > 0 else np.empty((0, 2))
        
        # Filter & sort cones ahead of car
        y_filtered = self._filter_and_sort_cones(y_np)
        b_filtered = self._filter_and_sort_cones(b_np)

        if len(y_filtered) == 0 or len(b_filtered) == 0:
            return None

        # greedy cone sorting - for each sorted yellow, find closest blue
        midpoints, gates, lone_flags = self._pair_and_midpoint(y_filtered, b_filtered)

        if len(midpoints) == 0:
            return None

        # Build waypoint list starting from car. A single gate ahead is enough to move towards:
        # on widely spaced tracks (acceleration: 5 m) the second gate is often not seen from
        # standstill, and requiring two kept the car from ever starting.
        waypoints = [np.array([0.0, 0.0])]
        waypoints.extend(self._chain_from_car(midpoints, gates))
        # One-boundary waypoints follow a single cone line, so they do not zig-zag like gate
        # midpoints do, and averaging them with their neighbours only pulls a tight corner's path
        # inwards (0.4 m in the mppi_track hairpin, onto the unseen inner cones): keep them as is.
        lone = {tuple(np.round(m, 6)) for m, l in zip(midpoints, lone_flags) if l}
        keep = [tuple(np.round(w, 6)) in lone for w in waypoints]
        if len(waypoints) < 2 or (len(waypoints) == 2 and waypoints[1][0] < 1.0):
            return None

        waypoints = self._smooth_path(waypoints, keep=keep)
        waypoints = [wp for wp in waypoints if wp[0] >= 0.0] # ahead of the car
        
        if len(waypoints) < 2:
            return None

        return np.array(waypoints)

    def _filter_and_sort_cones(self, cones):
        """Filter cones ahead of car and sort by forward distance"""
        if len(cones) == 0:
            return np.empty((0, 2))
        
        # x > min_lookahead and within lookahead distance
        mask = (cones[:, 0] > self.min_x_lookahead) & \
               (np.linalg.norm(cones, axis=1) < self.lookahead_distance)
        filtered = cones[mask]
        
        if len(filtered) == 0:
            return np.empty((0, 2))

        order = np.argsort(filtered[:, 0])
        return filtered[order]

    def _predict_next_midpoint(self) -> np.ndarray:
        """
        Takes last 2 midpoints gives next midpoint, assuming no directional change
        """
        if len(self.path_buffer) < 2:
            return errno.EINVAL
        p0 = self.path_buffer[0]
        p1 = self.path_buffer[1]
        return 2*p0 - p1
            

    def _pair_and_midpoint(self, yellow_cones, blue_cones):
        """
        Pair yellow and blue cones to form gates.
        Strategy: each yellow cone with its closest blue cone AND each blue cone with its
        closest yellow cone, by plain distance (cones may be reused), then drop duplicates.
        Returns (midpoints, gates, lone): the waypoints, each one's yellow -> blue gate vector (for
        the direction check in _chain_from_car) and whether it was placed from one boundary only.

        The old version paired each yellow with the closest *unused* blue at the same forward
        x (|dx| < cone_vertical_tolerance). In corners the gate cones are not side by side in
        the car frame, so it paired nothing (or zig-zagged, leaving the outer cones unused) and
        the car stopped mid-corner.
        """
        midpoints = []
        gates = []      # (yellow -> blue) vector of each midpoint's gate
        lone_mids = []  # per midpoint: placed from one boundary only (see below)
        width = self._track_width(yellow_cones, blue_cones)
        for side, other, is_yellow in ((yellow_cones, blue_cones, True), (blue_cones, yellow_cones, False)):
            if len(other) == 0:
                continue
            for c in side:
                dists = np.linalg.norm(other - c, axis=1)
                j = int(np.argmin(dists))
                # no gate partner within max_gate_distance: still usable as a one-boundary waypoint
                # (below) if it is near the car; the far cone then only tells which side the track is on
                far = dists[j] >= self.max_gate_distance
                if far and (dists[j] > self.lone_partner_max or np.linalg.norm(c) > self.lone_cone_range):
                    continue
                across = other[j] - c
                lone = False
                if dists[j] > self.lone_cone_factor * width:
                    # The closest cone of the other colour is much further than the track is wide:
                    # this cone's partner is not in the map (inner cones of a hairpin are outside
                    # the camera's and the lidar's view when the car passes them). The midpoint of
                    # such a skewed "gate" lies on the unseen boundary, so place the waypoint half
                    # a track width from this cone, square to its own boundary line instead.
                    normal = self._boundary_normal(c, side, across)
                    if normal is not None:
                        # stay a little closer to the boundary that is mapped (the controller's cone
                        # cost guards that one; nothing guards the unseen one)
                        across = normal * (width - 2.0 * self.lone_cone_bias)
                        # a waypoint this close to a mapped cone is not on the track: the
                        # "boundary line" was bent by a ghost or duplicate landmark
                        mid = c + across / 2.0
                        nearest = min(np.linalg.norm(side - mid, axis=1).min(), np.linalg.norm(other - mid, axis=1).min())
                        if nearest < self.min_cone_clearance * width:
                            continue
                        lone = True
                    elif far:
                        continue
                mid = c + across / 2.0
                if all(np.linalg.norm(mid - m) > 0.5 for m in midpoints):
                    midpoints.append(mid)
                    gates.append(across if is_yellow else -across)
                    lone_mids.append(lone)

        return midpoints, gates, lone_mids

    def _track_width(self, yellow_cones, blue_cones):
        """Track width from the cones in view: the lower quartile of every cone's distance to the
        closest cone of the other colour (2.5-6 m), else the default.

        Cones whose partner is missing or that pair diagonally are further from the other colour
        than the track is wide, never closer, so the low end of the distribution is the width (the
        median of the mutually-closest pairs gave 5.5 m at the hairpin and put one-boundary
        waypoints 2.5 m from the cone line, next to the other boundary)."""
        if len(yellow_cones) + len(blue_cones) < 6 or len(yellow_cones) == 0 or len(blue_cones) == 0:
            return self.default_track_width
        d = np.linalg.norm(yellow_cones[:, None, :] - blue_cones[None, :, :], axis=2)
        nearest = np.concatenate([d.min(axis=1), d.min(axis=0)])
        return float(np.clip(np.percentile(nearest, 25), 2.5, 6.0))

    def _boundary_normal(self, cone, same, towards, max_neighbour=6.0):
        """Unit vector square to the boundary line through `cone` (direction from its nearest
        same-colour neighbours), on the side `towards` points to. None if the cone has no neighbour."""
        d = np.linalg.norm(same - cone, axis=1)
        near = [i for i in np.argsort(d) if 1.0 < d[i] < max_neighbour][:2]   # > 1 m: not a duplicate of this cone
        if not near:
            return None
        if len(near) == 2 and np.dot(same[near[0]] - cone, same[near[1]] - cone) < 0:
            tangent = same[near[1]] - same[near[0]]      # one neighbour on each side
        else:
            tangent = same[near[0]] - cone
        n = np.array([-tangent[1], tangent[0]]) / (np.linalg.norm(tangent) + 1e-9)
        return n if np.dot(n, towards) >= 0 else -n

    def _chain_from_car(self, midpoints, gates):
        """Order midpoints by walking from the car to the nearest next one.

        Sorting by forward x breaks in hairpins (the track comes back towards the car). Each step
        must be shorter than max_step and turn less than max_turn, so midpoints from another part
        of the track are not joined in. `gates` holds each midpoint's yellow -> blue vector.
        """
        max_step, max_turn = self.chain_max_step, self.chain_max_turn
        remaining = [(np.asarray(m, dtype=float), np.asarray(g, dtype=float)) for m, g in zip(midpoints, gates)]
        pos, heading = np.zeros(2), 0.0
        chain = []
        while remaining:
            best, best_d = None, max_step
            for i, (m, gate) in enumerate(remaining):
                d = np.linalg.norm(m - pos)
                if d < 1e-3 or d >= best_d:
                    continue
                turn = math.atan2(m[1] - pos[1], m[0] - pos[0]) - heading
                if abs(math.atan2(math.sin(turn), math.cos(turn))) >= max_turn:
                    continue
                # Driving direction: blue on the left, yellow on the right. The step must cross the
                # gate that way (sin of the angle from step to yellow->blue > 0.1), which rules out
                # driving the track backwards and "gates" made across the infield. (0.5 rejected
                # valid steps near hairpin apexes, where the next gate is crossed at a shallow angle.)
                step = m - pos
                if (step[0] * gate[1] - step[1] * gate[0]) / (np.linalg.norm(step) * np.linalg.norm(gate) + 1e-9) > 0.1:
                    best, best_d = i, d
            if best is None:
                break
            m, _ = remaining.pop(best)
            heading = math.atan2(m[1] - pos[1], m[0] - pos[0])
            pos = m
            chain.append(m)
        return chain

    def _smooth_path(self, waypoints, window_size=None, keep=None):
        """
        Smooth waypoints using moving average filter (waypoints flagged in `keep` are not moved)
        """
        if window_size is None:
            window_size = self.smoothing_window
        
        if len(waypoints) <= window_size:
            return waypoints
        
        waypoints = np.array(waypoints)
        smoothed = [waypoints[0]]
        
        for i in range(1, len(waypoints) - 1):
            start = max(0, i - window_size // 2)
            end = min(len(waypoints), i + window_size // 2 + 1)
            
            window = waypoints[start:end]
            smoothed_point = waypoints[i] if keep is not None and keep[i] else np.mean(window, axis=0)
            smoothed.append(smoothed_point)
        
        smoothed.append(waypoints[-1])
        return smoothed
