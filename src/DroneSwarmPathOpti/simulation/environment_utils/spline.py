import numpy as np
from scipy.interpolate import CubicSpline

class CubicBSpline:
    """
    This class realizes the internal logic of paths which are built using Cubic-B-Splines.
    """

    t: np.ndarray # Time component
    x: CubicSpline # X component
    y: CubicSpline # Y component

    raw_path: list[tuple[float, float, float]]

    def __init__(self, path: list[tuple[float, float, float]]):
        self.raw_path = path

        timestamps: list[float] = [0]
        for i in range(1, len(path)):
            x0, y0, v0 = path[i - 1]
            x1, y1, v1 = path[i]
            distance: float = np.hypot(x1 - x0, y1 - y0)  # Euclidian distance
            average_velocity: float = (v0 + v1) / 2
            delta_time: float = distance / average_velocity # Time: distance / velocity
            if delta_time <= 0: # Enforce positive progression
                delta_time += 1e-6
            timestamps.append(timestamps[-1] + delta_time)

        t_temp = np.array(timestamps)

        if not np.all(np.diff(t_temp) > 0):
            raise ValueError(f"Non-increasing time values in spline path: {t_temp}")

        self.t = t_temp
        x_temp = np.array([p[0] for p in path])
        y_temp = np.array([p[1] for p in path])

        self.x = CubicSpline(t_temp, x_temp) # Interpolate X-movement
        self.y = CubicSpline(t_temp, y_temp) # Interpolate Y-movement

    def calculate_energy_usage(self, resolution: int = 50, alpha: float = 1.0, beta: float = 0.1) -> float:
        """
        Compute the estimated energy consumption along a 2D path based on velocity and acceleration profiles.

        This method numerically integrates an energy-like quantity over time, derived from the
        velocity and acceleration of the path. The function assumes the path is represented by
        parametric spline functions `x(t)` and `y(t)` with continuous derivatives. The resulting
        energy usage is computed as the time integral of a weighted sum of velocity and squared
        acceleration.

        :param resolution: Number of evenly spaced samples over the path duration used for numerical integration. Higher values yield more accurate results at the cost of performance.
        :param alpha: Weighting factor for the velocity-dependent energy term.
        :param beta: Weighting factor for the acceleration-dependent energy term.
        :return: Estimated total energy usage along the path (in arbitrary energy units).
        """
        ts = np.linspace(self.t[0], self.t[-1], resolution)

        dxdt = self.x.derivative()(ts)
        dydt = self.y.derivative()(ts)
        v = np.sqrt(dxdt ** 2 + dydt ** 2)

        d2xdt2 = self.x.derivative(2)(ts)
        d2ydt2 = self.y.derivative(2)(ts)
        a_squared = d2xdt2 ** 2 + d2ydt2 ** 2

        power = alpha * v + beta * a_squared

        energy = np.trapezoid(power, ts)
        return energy

    def calculate_time_usage(self) -> float:
        """
        Compute the estimated time consumption along a 2D path based on velocity and acceleration.

        :return: Total time usage along the path (in arbitrary time units).
        """
        return float(self.t[-1])