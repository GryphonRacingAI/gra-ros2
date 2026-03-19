from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ProcessedPath:
    points: np.ndarray      # (M, 2) interpolated xy coordinates
    heading: np.ndarray     # (M,)   reference heading at each point
    speed_ref: np.ndarray   # (M,)   curvature-limited reference speed


@dataclass(frozen=True)
class PathProcessorConfig:
    a_lat_max: float = 2.0
    v_max_straight: float = 7.0
    v_min: float = 1.0
    num_between: int = 5


class PathProcessor:
    def __init__(self, config: PathProcessorConfig):
        self._cfg = config

    def __call__(self, waypoints: np.ndarray) -> ProcessedPath:
        """Process (N,2) waypoints into interpolated path with speed profile and heading."""
        if waypoints.ndim != 2 or waypoints.shape[1] != 2:
            raise ValueError("waypoints must be an (N, 2) array")
        if waypoints.shape[0] < 2:
            raise ValueError("At least 2 waypoints are required")

        path_pts = _interpolate_points(waypoints, num_between=self._cfg.num_between)
        path_x = path_pts[:, 0]
        path_y = path_pts[:, 1]

        dx_p = np.gradient(path_x)
        dy_p = np.gradient(path_y)
        ddx_p = np.gradient(dx_p)
        ddy_p = np.gradient(dy_p)

        kappa = (dx_p * ddy_p - dy_p * ddx_p) / (dx_p**2 + dy_p**2 + 1e-6)**1.5
        kappa_abs = np.abs(kappa)

        v_ref = np.sqrt(self._cfg.a_lat_max / (kappa_abs + 1e-3))
        v_ref = np.clip(v_ref, self._cfg.v_min, self._cfg.v_max_straight)

        heading_ref = np.arctan2(dy_p, dx_p)

        return ProcessedPath(points=path_pts, heading=heading_ref, speed_ref=v_ref)


def _interpolate_points(arr: np.ndarray, num_between: int = 3) -> np.ndarray:
    result = []

    for i in range(len(arr) - 1):
        start = arr[i]
        end = arr[i + 1]

        # generate num_between + 2 points including start/end
        segment = np.linspace(start, end, num_between + 2)

        # avoid duplicating the start point except for the first segment
        if i > 0:
            segment = segment[1:]

        result.extend(segment)

    return np.array(result)
