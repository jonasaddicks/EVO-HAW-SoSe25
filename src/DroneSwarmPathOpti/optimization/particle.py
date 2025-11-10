import math
import numpy as np

from DroneSwarmPathOpti.config import get_settings
from copy import deepcopy

settings = get_settings()
rng = np.random.default_rng(settings.SEED_PARTICLE)



class DronePath:
    """
    This class represents a drone's path.

    A drone's path consists on a fix number of points (specified in the config file) inside the environment.
    Every point in a path consists of:
        - X-coordinate
        - Y-coordinate
        - Velocity (The velocity a drone will have when passing this point)
    """

    control_points: list[tuple[float, float, float]]

    def __init__(self, control_points: list[tuple[float, float, float]]):
        self.control_points = control_points # List of tuples (x -> X-coordinate, y -> Y-coordinate, v -> drone velocity)

    def get_positions(self) -> list[tuple[float, float]]:
        """
        Returns the spacial positions of the drone's path.

        :return: List of tuples containing x-coordinates and y-coordinates of the drone's path.
        """
        return [(x, y) for x, y, _ in self.control_points]

    def get_velocities(self) -> list[float]:
        """
        Returns the velocities of the drone's path.

        :return: List of floats containing all the velocities of the drone at each point.
        """
        return [v for _, _, v in self.control_points]


class Particle:
    """
    This class represents a particle and bundles a fix amount of DronePaths (specified in the config file).
    """

    num_control_points: int # Number of points in a drone's path
    map_bounds: tuple[float, float] # Size of the environment
    map_start: tuple[float, float] # Position of the start
    map_goal: tuple[float, float] # Position of the goal
    max_drone_speed: float # Physical cap on a drone's velocity

    particle_position: list[DronePath] # NOTE: this represents the particle's current position, NOT any drone's position (The current solution inside the solution space)
    particle_velocity: list[DronePath] # NOTE: this represents the velocity of the particle, NOT the velocity of any drone (The current velocity with which the particle explores the solution space)

    best_position: list[DronePath] # 'best position' this particle has found (so far)
    best_fitness: float # best fitness value this particle has found (so far)
    current_fitness: float # fitness value of the current position

    velocity_damping: float # damping value of the particles velocity after 'violating the boundaries'

    def __init__(self):
        self.num_drones = settings.NUMBER_DRONES
        self.num_control_points = settings.INITIAL_CONTROL_POINTS
        self.map_bounds = (settings.ENVIRONMENT_SIZE_X, settings.ENVIRONMENT_SIZE_Y)  # (x_max, y_max)
        self.map_start = (settings.START_X, settings.START_Y)
        self.map_goal = (settings.GOAL_X, settings.GOAL_Y)
        self.max_drone_speed = settings.DRONE_MAX_SPEED

        self.initial_position_bounds = settings.PSO_INITIAL_POSITION_BOUNDS
        self.initial_distance_paths = settings.PSO_INITIAL_DISTANCE_PATHS

        self.velocity_damping = settings.PSO_VELOCITY_DAMPING

        self.particle_position = self._initialize_position()
        self.particle_velocity = self._initialize_velocity()

        self.best_position = deepcopy(self.particle_position)
        self.best_fitness = float('inf')
        self.current_fitness = float('inf')

    def _initialize_position(self): # TODO a lot of this can be done just once instead of every time a particle is initialized
        """
        This method initializes the first position of a particle.

        :return: A randomized position inside the solution space represented by a fix amount of drone paths.
        """
        # Vector start -> goal
        vec_start_goal: np.array = np.array([
            self.map_goal[0] - self.map_start[0],
            self.map_goal[1] - self.map_start[1]
        ])
        # Vector perpendicular to vector start -> goal
        vec_perpendicular_start_goal: np.array = np.array([
            vec_start_goal[1],
            -vec_start_goal[0]
        ])

        # Perpendicular distance between two most outer drone paths
        distance_perpendicular: float = (self.num_drones - 1) * self.initial_distance_paths

        # Distance start -> goal
        distance_start_goal: float = math.sqrt((self.map_goal[0] - self.map_start[0])**2 + (self.map_goal[1] - self.map_start[1])**2)

        # Point in the middle of vector start -> goal
        point_center_start_goal: np.array = np.array ([
            self.map_start[0] + vec_start_goal[0] * 0.5,
            self.map_start[1] + vec_start_goal[1] * 0.5
        ])

        # Normalized vector perpendicular to vector start -> goal
        vec_normalized_perpendicular_start_goal: np.array = vec_perpendicular_start_goal / np.linalg.norm(vec_perpendicular_start_goal)

        # Normalized vector start -> goal
        vec_normalized_start_goal: np.array = vec_start_goal / np.linalg.norm(vec_start_goal)

        # Point in the middle of the most outer drone path
        point_anchor_path: np.array = point_center_start_goal + distance_perpendicular * 0.5 * vec_normalized_perpendicular_start_goal

        # Portion of the distance start -> goal each drone path has
        peak_profile: list[float] = self._generate_peak_profile(self.num_drones)

        # Point on the lower end of the most outer drone path
        point_anchor_control_point: np.array = point_anchor_path + distance_start_goal * 0.5 * peak_profile[0] * -vec_normalized_start_goal

        # Distance between each control_point on the path
        distance_control_points: float = distance_start_goal * peak_profile[0] / (self.num_drones + 1)

        initial_position: list[DronePath] = []
        for i in range(self.num_drones):
            distance_control_points = distance_start_goal * peak_profile[i] / (self.num_control_points + 1)  # Distance between each control_point on the path

            control_points: list[tuple[float, float, float]] = []
            for _ in range(self.num_control_points): # Initialize every point of a drone path inside its bounds
                point_anchor_control_point = point_anchor_control_point + distance_control_points * vec_normalized_start_goal # Progress to next control point
                control_points.append((
                    rng.uniform(
                        float(point_anchor_control_point[0] - self.initial_position_bounds / 2),
                        float(point_anchor_control_point[0] + self.initial_position_bounds / 2)),  # X-coordinate
                    rng.uniform(
                        float(point_anchor_control_point[1] - self.initial_position_bounds / 2),
                        float(point_anchor_control_point[1] + self.initial_position_bounds / 2)),  # Y-coordinate
                    rng.uniform(0.0, float(self.max_drone_speed))  # Drone velocity
                ))
            initial_position.append(DronePath(control_points))

            point_anchor_path = point_anchor_path + self.initial_distance_paths * -vec_normalized_perpendicular_start_goal # Move to the next drone path
            point_anchor_control_point = point_anchor_path + distance_start_goal * 0.5 * peak_profile[i+1] * -vec_normalized_start_goal # Reset the anchor of the control points

        return initial_position

    def _initialize_velocity(self):
        """
        This method initializes the initial velocity inside the by the config specified bounds.

        :return: A randomized velocity for the bounds specified by the config represented by a fix amount of drone paths.
        """
        return [
            DronePath([
                (
                    rng.uniform(-settings.PSO_MAX_INITIAL_VELOCITY_X, settings.PSO_MAX_INITIAL_VELOCITY_X),  # dx
                    rng.uniform(-settings.PSO_MAX_INITIAL_VELOCITY_Y, settings.PSO_MAX_INITIAL_VELOCITY_Y),  # dy
                    rng.uniform(-settings.PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY, settings.PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY)   # dv
                )
                for _ in range(self.num_control_points)
            ])
            for _ in range(self.num_drones)
        ]

    def update_velocity(self, global_best_position: list[DronePath]) -> None:
        """
        This method updates the particles velocity by taking current velocity, current personal best and the current global best into consideration.

        :param global_best_position: the best position found (so far) by all competing particles.
        :return: None
        """
        for drone in range(self.num_drones):
            for control_point in range(self.num_control_points):
                current_position_x, current_position_y, current_position_v = self.particle_position[drone].control_points[control_point]
                velocity_x, velocity_y, velocity_v = self.particle_velocity[drone].control_points[control_point]
                personal_best_x, personal_best_y, personal_best_v = self.best_position[drone].control_points[control_point]
                global_best_x, global_best_y, global_best_v = global_best_position[drone].control_points[control_point]

                random_factor_personal_best, random_factor_global_best = rng.uniform(0, 1), rng.uniform(0, 1)

                new_vx = (
                        settings.PSO_WEIGHT_PERSONAL_POSITION * velocity_x
                        + settings.PSO_WEIGHT_PERSONAL_BEST * random_factor_personal_best * (personal_best_x - current_position_x)
                        + settings.PSO_WEIGHT_GLOBAL_BEST * random_factor_global_best * (global_best_x - current_position_x)
                )
                new_vy = (
                        settings.PSO_WEIGHT_PERSONAL_POSITION * velocity_y
                        + settings.PSO_WEIGHT_PERSONAL_BEST * random_factor_personal_best * (personal_best_y - current_position_y)
                        + settings.PSO_WEIGHT_GLOBAL_BEST * random_factor_global_best * (global_best_y - current_position_y)
                )
                new_vv = (
                        settings.PSO_WEIGHT_PERSONAL_POSITION * velocity_v
                        + settings.PSO_WEIGHT_PERSONAL_BEST * random_factor_personal_best * (personal_best_v - current_position_v)
                        + settings.PSO_WEIGHT_GLOBAL_BEST * random_factor_global_best * (global_best_v - current_position_v)
                )

                new_vx = self.clip(new_vx, -settings.PSO_MAX_VELOCITY_X, settings.PSO_MAX_VELOCITY_X)
                new_vy = self.clip(new_vy, -settings.PSO_MAX_VELOCITY_Y, settings.PSO_MAX_VELOCITY_Y)
                new_vv = self.clip(new_vv, -settings.PSO_MAX_VELOCITY_DRONE_VELOCITY, settings.PSO_MAX_VELOCITY_DRONE_VELOCITY)

                self.particle_velocity[drone].control_points[control_point] = (new_vx, new_vy, new_vv)

    def update_position(self) -> None:
        """
        This method updates the particle's current position using the particle's current velocity.

        :return: None
        """
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
                    new_v = self.clip(v + dv, 0.1, self.max_drone_speed)  # drone's velocity always positive but below max drone speed
                    dv *= -self.velocity_damping

                self.particle_velocity[drone].control_points[control_point] = (dx, dy, dv)
                self.particle_position[drone].control_points[control_point] = (new_x, new_y, new_v)

    def reset_velocity(self) -> None:
        """
        This method resets the particle's current velocity by randomizing a new velocity.

        :return: None
        """
        self.particle_velocity = [
            DronePath([
                (
                    rng.uniform(-settings.PSO_MAX_INITIAL_VELOCITY_X, settings.PSO_MAX_INITIAL_VELOCITY_X),  # dx
                    rng.uniform(-settings.PSO_MAX_INITIAL_VELOCITY_Y, settings.PSO_MAX_INITIAL_VELOCITY_Y),  # dy
                    rng.uniform(-settings.PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY, settings.PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY)  # dv
                )
                for _ in range(self.num_control_points)
            ])
            for _ in range(self.num_drones)
        ]

    @staticmethod
    def clip(value: float, lower: float, upper: float) -> float:
        """
        Clamp a numeric value to a specified range.

        Ensures that the returned value does not fall below the lower bound or
        exceed the upper bound. If `value` is less than `lower`, `lower` is returned.
        If `value` is greater than `upper`, `upper` is returned. Otherwise, `value`
        itself is returned.

        :param value: The numeric value to be clamped.
        :param lower: The minimum allowed value (lower bound of the range).
        :param upper: The maximum allowed value (upper bound of the range).
        :return: The clamped value, guaranteed to lie within [lower, upper].
        """
        return max(min(value, upper), lower)

    @staticmethod
    def _generate_peak_profile(n: int, step: float = 0.2) -> list[float]:
        """
        Generate a symmetric peak-shaped numeric profile.

        Creates a list of floating-point values representing a symmetric peak that rises
        toward the center and then falls off, followed by a trailing zero. The height of
        the peak and the smoothness of the slope are controlled by the `step` parameter.

        For an odd `n`, the profile has a single central peak of 1.0.
        For an even `n`, the profile is symmetric around the center

        :param n: The total number of values in the profile (excluding the trailing zero).
        :param step: The increment/decrement between consecutive values on each side of the peak.
        :return: A list of floats representing the generated peak profile, ending with 0.0.
        """
        half = n // 2
        base = [1 - (i * step) for i in range(half, 0, -1)]
        if n % 2 == 0:
            profile = [i + step for i in base + base[::-1]] + [0.0]
        else:
            profile = base + [1.0] + base[::-1] + [0.0]
        return profile

    def _initialize_position_temp(self): # TODO a lot of this can be done just once instead of every time a particle is initialized
        # Vector start -> goal
        vec_start_goal: np.array = np.array([
            self.map_goal[0] - self.map_start[0],
            self.map_goal[1] - self.map_start[1]
        ])
        # Vector perpendicular to vector start -> goal
        vec_perpendicular_start_goal: np.array = np.array([
            vec_start_goal[0],
            -vec_start_goal[1]
        ])

        # Perpendicular distance between two most outer drone paths
        distance_perpendicular: float = (self.num_drones - 1) * self.initial_distance_paths

        # Distance start -> goal
        distance_start_goal: float = math.sqrt((self.map_goal[0] - self.map_start[0])**2 + (self.map_goal[1] - self.map_start[1])**2)

        # Point in the middle of vector start -> goal
        point_center_start_goal: np.array = np.array ([
            self.map_start[0] + vec_start_goal[0] * 0.5,
            self.map_start[1] + vec_start_goal[1] * 0.5
        ])

        # Normalized vector perpendicular to vector start -> goal
        vec_normalized_perpendicular_start_goal: np.array = vec_perpendicular_start_goal / np.linalg.norm(vec_perpendicular_start_goal)

        # Normalized vector start -> goal
        vec_normalized_start_goal: np.array = vec_start_goal / np.linalg.norm(vec_start_goal)

        # Point in the middle of the most outer drone path
        point_anchor_path: np.array = point_center_start_goal + distance_perpendicular * 0.5 * vec_normalized_perpendicular_start_goal

        # Portion of the distance start -> goal each drone path has
        peak_profile: list[float] = self._generate_peak_profile(self.num_drones)

        # Point on the lower end of the most outer drone path
        point_anchor_control_point: np.array = point_anchor_path + distance_start_goal * 0.5 * peak_profile[0] * -vec_normalized_start_goal

        # Distance between each control_point on the path
        distance_control_points: float = distance_start_goal * peak_profile[0] / (self.num_drones + 1)

        initial_position: list[tuple[float, float]] = []
        for i in range(self.num_drones):
            distance_control_points = distance_start_goal * peak_profile[i] / (self.num_control_points + 1)  # Distance between each control_point on the path

            for _ in range(self.num_control_points):
                point_anchor_control_point = point_anchor_control_point + distance_control_points * vec_normalized_start_goal # Progress to next control point
                initial_position.append(point_anchor_control_point)

            point_anchor_path = point_anchor_path + self.initial_distance_paths * -vec_normalized_perpendicular_start_goal # Move to the next drone path
            point_anchor_control_point = point_anchor_path + distance_start_goal * 0.5 * peak_profile[i+1] * -vec_normalized_start_goal # Reset the anchor of the control points

        return initial_position