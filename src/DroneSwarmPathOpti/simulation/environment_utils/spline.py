import numpy as np
from scipy.interpolate import CubicSpline

class CubicBSpline:

    t: np.ndarray
    x: CubicSpline
    y: CubicSpline

    def __init__(self, path: list[tuple[float, float, float]]):
        distances = [0]
        for i in range(1, len(path)):
            x0, y0, v0 = path[i - 1]
            x1, y1, v1 = path[i]
            distance = np.hypot(x1 - x0, y1 - y0)  # Euclidian distance
            average_velocity = (v0 + v1) / 2
            delta_time = distance / average_velocity if average_velocity > 0 else distance  # Time: distance / velocity
            distances.append(distances[-1] + delta_time)

        t_temp = np.array(distances)
        x_temp = np.array([p[0] for p in path])
        y_temp = np.array([p[1] for p in path])

        self.t = np.array(distances)
        self.x = CubicSpline(t_temp, x_temp)
        self.y = CubicSpline(t_temp, y_temp)