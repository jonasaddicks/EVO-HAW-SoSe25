import numpy as np

from DroneSwarmPathOpti.config import get_settings
from copy import deepcopy

settings = get_settings()

class DronePath:

    control_points: list[tuple[float, float, float]]

    def __init__(self, control_points: list[tuple[float, float, float]]):
        self.control_points = control_points # List of tuples (x, y, v)

    def get_positions(self) -> list[tuple[float, float]]:
        return [(x, y) for x, y, _ in self.control_points]

    def get_velocities(self) -> list[float]:
        return [v for _, _, v in self.control_points]

# TODO
class Particle:

    def __init__(self, num_drones: int, num_control_points: int, map_bounds: tuple[int, int]):
        self.num_drones = num_drones
        self.num_control_points = num_control_points
        self.map_bounds = map_bounds  # ((xmin, xmax), (ymin, ymax))

        self.position = self._initialize_position()
        self.velocity = self._initialize_velocity()

        self.best_position = deepcopy(self.position)
        self.best_fitness = float('inf')
        self.current_fitness = None

    def _initialize_position(self):
        xmin, xmax = self.map_bounds[0]
        ymin, ymax = self.map_bounds[1]

        return [
            DronePath([
                (
                    np.random.uniform(xmin, xmax),
                    np.random.uniform(ymin, ymax),
                    np.random.uniform(1.0, 5.0)
                )
                for _ in range(self.num_control_points)
            ])
            for _ in range(self.num_drones)
        ]

    def _initialize_velocity(self):
        return [
            DronePath([
                (
                    np.random.uniform(-1.0, 1.0),  # dx
                    np.random.uniform(-1.0, 1.0),  # dy
                    np.random.uniform(-0.5, 0.5)   # dv
                )
                for _ in range(self.num_control_points)
            ])
            for _ in range(self.num_drones)
        ]

    def update_position(self):
        for d in range(self.num_drones):
            for i in range(self.num_control_points):
                x, y, v = self.position[d].control_points[i]
                dx, dy, dv = self.velocity[d].control_points[i]

                new_x = x + dx
                new_y = y + dy
                new_v = max(0.1, v + dv)  # Geschwindigkeit immer positiv

                self.position[d].control_points[i] = (new_x, new_y, new_v)

    def update_velocity(self, global_best_position, w=0.5, c1=1.5, c2=1.5):
        for d in range(self.num_drones):
            for i in range(self.num_control_points):
                px, py, pv = self.position[d].control_points[i]
                vx, vy, vv = self.velocity[d].control_points[i]
                pbest_x, pbest_y, pbest_v = self.best_position[d].control_points[i]
                gbest_x, gbest_y, gbest_v = global_best_position[d].control_points[i]

                r1, r2 = np.random.rand(), np.random.rand()

                new_vx = (
                    w * vx
                    + c1 * r1 * (pbest_x - px)
                    + c2 * r2 * (gbest_x - px)
                )
                new_vy = (
                    w * vy
                    + c1 * r1 * (pbest_y - py)
                    + c2 * r2 * (gbest_y - py)
                )
                new_vv = (
                    w * vv
                    + c1 * r1 * (pbest_v - pv)
                    + c2 * r2 * (gbest_v - pv)
                )

                self.velocity[d].control_points[i] = (new_vx, new_vy, new_vv)
