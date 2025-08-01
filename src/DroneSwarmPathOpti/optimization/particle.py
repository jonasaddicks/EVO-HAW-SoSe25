import numpy as np

from DroneSwarmPathOpti.config import get_settings
from copy import deepcopy

settings = get_settings()

class DronePath:

    control_points: list[tuple[float, float, float]]

    def __init__(self, control_points: list[tuple[float, float, float]]):
        self.control_points = control_points # List of tuples (x -> X-coordinate, y -> Y-coordinate, v -> drone velocity)

    def get_positions(self) -> list[tuple[float, float]]:
        return [(x, y) for x, y, _ in self.control_points]

    def get_velocities(self) -> list[float]:
        return [v for _, _, v in self.control_points]


class Particle:

    num_control_points: int
    map_bounds: tuple[float, float]
    max_drone_speed: float

    particle_position: list[DronePath]
    particle_velocity: list[DronePath]

    best_position: list[DronePath]
    best_fitness: float
    current_fitness: float

    max_initial_velocity_x: float
    max_initial_velocity_y: float
    max_initial_velocity_drone_velocity: float
    velocity_damping: float

    max_velocity_x: float
    max_velocity_y: float
    max_velocity_drone_velocity: float

    weight_personal_position: float
    weight_personal_best: float
    weight_global_best: float

    def __init__(self):
        self.num_drones = settings.NUMBER_DRONES
        self.num_control_points = settings.INITIAL_CONTROL_POINTS
        self.map_bounds = (settings.ENVIRONMENT_SIZE_X, settings.ENVIRONMENT_SIZE_Y)  # (x_max, y_max)
        self.max_drone_speed = settings.DRONE_MAX_SPEED

        self.max_initial_velocity_x = settings.PSO_MAX_INITIAL_VELOCITY_X
        self.max_initial_velocity_y = settings.PSO_MAX_INITIAL_VELOCITY_Y
        self.max_initial_velocity_drone_velocity = settings.PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY
        self.velocity_damping = settings.PSO_VELOCITY_DAMPING

        self.max_velocity_x = settings.PSO_MAX_VELOCITY_X
        self.max_velocity_y = settings.PSO_MAX_VELOCITY_Y
        self.max_velocity_drone_velocity = settings.PSO_MAX_VELOCITY_DRONE_VELOCITY

        self.weight_personal_position = settings.PSO_WEIGHT_PERSONAL_POSITION
        self.weight_personal_best = settings.PSO_WEIGHT_PERSONAL_BEST
        self.weight_global_best = settings.PSO_WEIGHT_GLOBAL_BEST

        self.particle_position = self._initialize_position()
        self.particle_velocity = self._initialize_velocity()

        self.best_position = deepcopy(self.particle_position)
        self.best_fitness = float('inf')
        self.current_fitness = float('inf')

    def _initialize_position(self): # TODO initialize points in quadrants
        x_min, x_max = 0, self.map_bounds[0]
        y_min, y_max = 0, self.map_bounds[1]

        return [
            DronePath([
                (
                    np.random.uniform(x_min, x_max), # X-coordinate
                    np.random.uniform(y_min, y_max), # Y-coordinate
                    np.random.uniform(0.0, self.max_drone_speed) # Drone velocity
                )
                for _ in range(self.num_control_points)
            ])
            for _ in range(self.num_drones)
        ]

    def _initialize_velocity(self):
        return [
            DronePath([
                (
                    np.random.uniform(-self.max_initial_velocity_x, self.max_initial_velocity_x),  # dx
                    np.random.uniform(-self.max_initial_velocity_y, self.max_initial_velocity_y),  # dy
                    np.random.uniform(-self.max_initial_velocity_drone_velocity, self.max_initial_velocity_drone_velocity)   # dv
                )
                for _ in range(self.num_control_points)
            ])
            for _ in range(self.num_drones)
        ]

    def update_velocity(self, global_best_position: list[DronePath]):
        for drone in range(self.num_drones):
            for control_point in range(self.num_control_points):
                current_position_x, current_position_y, current_position_v = self.particle_position[drone].control_points[control_point]
                velocity_x, velocity_y, velocity_v = self.particle_velocity[drone].control_points[control_point]
                personal_best_x, personal_best_y, personal_best_v = self.best_position[drone].control_points[control_point]
                global_best_x, global_best_y, global_best_v = global_best_position[drone].control_points[control_point]

                random_factor_personal_best, random_factor_global_best = np.random.rand(), np.random.rand()

                new_vx = (
                        self.weight_personal_position * velocity_x
                        + self.weight_personal_best * random_factor_personal_best * (personal_best_x - current_position_x)
                        + self.weight_global_best * random_factor_global_best * (global_best_x - current_position_x)
                )
                new_vy = (
                        self.weight_personal_position * velocity_y
                        + self.weight_personal_best * random_factor_personal_best * (personal_best_y - current_position_y)
                        + self.weight_global_best * random_factor_global_best * (global_best_y - current_position_y)
                )
                new_vv = (
                        self.weight_personal_position * velocity_v
                        + self.weight_personal_best * random_factor_personal_best * (personal_best_v - current_position_v)
                        + self.weight_global_best * random_factor_global_best * (global_best_v - current_position_v)
                )

                new_vx = self.clip(new_vx, -self.max_velocity_x, self.max_velocity_x)
                new_vy = self.clip(new_vy, -self.max_velocity_y, self.max_velocity_y)
                new_vv = self.clip(new_vv, -self.max_velocity_drone_velocity, self.max_velocity_drone_velocity)

                self.particle_velocity[drone].control_points[control_point] = (new_vx, new_vy, new_vv)

    def update_position(self):
        for drone in range(self.num_drones):
            for control_point in range(self.num_control_points):
                x, y, v = self.particle_position[drone].control_points[control_point]
                dx, dy, dv = self.particle_velocity[drone].control_points[control_point]

                new_x = x + dx
                if new_x < 0 or new_x > self.map_bounds[0]:
                    new_x = self.clip(new_x, 0, self.map_bounds[0])
                    dx *= -self.velocity_damping

                new_y = y + dy
                if new_y < 0 or new_y > self.map_bounds[1]:
                    new_y = self.clip(new_y, 0, self.map_bounds[1])
                    dy *= -self.velocity_damping

                new_v = v + dv
                if new_v < 0.1 or new_v > self.max_drone_speed:
                    new_v = self.clip(v + dv, 0.1, self.max_drone_speed)  # velocity always positive but below max drone speed
                    dv *= -self.velocity_damping

                self.particle_velocity[drone].control_points[control_point] = (dx, dy, dv)
                self.particle_position[drone].control_points[control_point] = (new_x, new_y, new_v)

    @staticmethod
    def clip(value: float, lower: float, upper: float) -> float:
        return max(min(value, upper), lower)