import numpy as np

from DroneSwarmPathOpti.config import get_settings

from .drone import Drone
from .map_object import MapObject
from .map_object import collision_objects
from ..environment_utils import traverse
from ...project_logger import log_info, Source, log_warning

settings = get_settings()
rng = np.random.default_rng(None if settings.SEED_ENVIRONMENT == -1 else settings.SEED_ENVIRONMENT)

class Obstacle(MapObject):
    """
    This class represents a single obstacle in an environment.
    The purpose of an obstacle is to make an environment unique and create a set of non-trivial paths from a start to a goal in an environment.
    """

    def __init__(self, position: tuple[int, int], base_radius: float):
        super().__init__(position, rng.uniform(base_radius - base_radius*0.5, base_radius*1.5))

class SingletonMeta(type):
    """
    This metaclass creates a singleton class to prohibit multiple instances of any inheriting classes.
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super().__call__(*args, **kwargs)
        return cls._instances[cls]

class Environment(metaclass=SingletonMeta):
    """
    This class represents the environment which contains all drones and obstacles to take part in the optimization process.
    The Environment is implemented as a singleton to prevent any parallel existing environment from instantiating.

    An Environment consists of:
        - Width and Height
        - A list of drones
        - A list of obstacles
        - Starting and goal points
    """
    bounds: tuple[int, int]
    drones: list[Drone]
    obstacles: list[Obstacle]
    start: MapObject | None
    goal: MapObject | None

    traversable:bool # True if there has to be at least one path from start to goal
    validation_path: list[tuple[int, int]] | None # None if not traversable

    def __init__(self,
                 bounds: tuple[int, int],
                 drones: list[Drone],
                 traversable: bool,
                 start: tuple[int, int]=None,
                 start_radius: int=5,
                 goal: tuple[int, int]=None,
                 goal_radius: int=5
                 ):

        self.start = MapObject(start, start_radius) if start is not None else None
        self.goal = MapObject(goal, goal_radius) if goal is not None else None

        self.traversable = traversable
        self.bounds = bounds
        self.drones = drones
        self.obstacles = []

    def generate_obstacles(self,
                           obstacles: int,
                           base_radius: float
                           ) -> bool:
        """
        This method generates a specified amount of obstacles, makes sure they don't collide with start or goal and adds them to the environment.
        If specified in the config, the generator will enforce a map that is traversable from start to goal.
        The generation process can fail in two scenarios:
            - The generator exceeded the permitted amount of collisions between obstacles and start/goal (which implies obstacles are too big)
            - The generator exceeded the permitted amount of retries after generating a non-traversable environment (which implies too many obstacles)

        :param obstacles: Number of obstacles to generate
        :param base_radius: Average radius of the obstacles to generate
        :return: Return true if obstacles were successfully generated, false otherwise
        """

        (x_max, y_max) = self.bounds
        tries_traversable: int = 0
        while True:
            obstacles_list: list[Obstacle] = []
            for _ in range(obstacles):
                obstacle: Obstacle
                tries_obstacle: int = 0

                while True:
                    x = rng.integers(0, x_max + 1)
                    y = rng.integers(0, y_max + 1)
                    obstacle = Obstacle((int(x), int(y)), base_radius)
                    if (
                            (self.start is None or self.goal is None)
                            or
                            not (collision_objects(obstacle, self.start) or collision_objects(obstacle, self.goal))):
                        break  # Exit loop if generated object collides neither with start nor goal
                    else:
                        tries_obstacle += 1
                        log_info(Source.environment, f'obstacle clipping with start or goal in try {tries_obstacle} - placing again')
                        if tries_obstacle >= 10:
                            log_warning(Source.environment, f'exceeded number of tries for placing an obstacle, obstacles remain empty')
                            return False # Exceeded maximum number of tries to generate an obstacle -> obstacles probably too big
                obstacles_list.append(obstacle)
            self.obstacles = obstacles_list

            if self.traversable and self.start is not None and self.goal is not None:
                path: list[tuple[int, int]] = self._validate_map() # Check if a map is traversable from start to goal
                if len(path) > 0: # Map is traversable
                    self.validation_path = path
                    break
                else:
                    tries_traversable += 1
                    log_info(Source.environment, f'failed to traverse map in try {tries_traversable} - regenerating obstacles')
                    if tries_traversable >= 10:
                        self.validation_path = [] # The Map should've been traversable, but the algorithm was not able to find a possible path with the given number of tries
                        log_warning(Source.environment, 'failed to find a valid path - max number of tries exceeded, path is empty')
                        return False # Exceeded maximum number of tries to traverse the map -> probably too many obstacles
            else: # Map must not be traversable or start/goal is none
                self.validation_path = None
                break
        return True

    def _validate_map(self) -> list[tuple[int, int]]:
        """
        This method checks if the environment is traversable or not by translating it into a gridlike graph and removing all nodes overlapping with an obstacle.

        :return: Return the path found from start to goal if the environment is traversable, an empty path otherwise
        """
        width, height = self.bounds
        grid_data = [[0 for _ in range(width)] for _ in range(height)]

        for obstacle in self.obstacles:
            ox, oy = obstacle.position
            r = int(obstacle.radius)
            for y in range(max(0, oy - r), min(height, oy + r)):
                for x in range(max(0, ox - r), min(width, ox + r)):
                    if (x - ox) ** 2 + (y - oy) ** 2 <= r ** 2:
                        grid_data[y][x] = 1 # Cut link

        start = self.start.position
        goal = self.goal.position

        path = traverse(grid_data, start, goal)
        return path

    def get_collisions_obstacles(self, resolution: float=1.0) -> list[tuple[int, int]]:
        """
        This method checks for collisions between drones and obstacles in the environment.
        By using interpolation with the drone's current path and radius, a list containing all collisions and their positions is generated.

        :param resolution: The resolution determines the size of the steps with which the collision detection should be performed on a drone's path. A lower resolution implies more collision checks will be performed.
        :return: A list of all collisions between drones and obstacles.
        """
        collisions_obstacles: list[tuple[int, int]] = []
        for drone in self.drones:

            t_samples = np.arange(drone.path.t[0], drone.path.t[-1], resolution) # Create an even distribution along the path of a drone
            for t in t_samples: # For every evenly distributed point in a drone's path
                x, y, r = drone.path.x(t), drone.path.y(t), drone.radius # Next step
                for obstacle in self.obstacles: # Check if a collision occurs at the currently reviewed position of a drone
                    if _collision_raw(obstacle.position[0], obstacle.position[1], obstacle.radius, x, y, r):
                        collisions_obstacles.append((x, y))
        return collisions_obstacles

    def get_collisions_drones(self, resolution: float=1.0) -> list[tuple[int, int]]:
        """
        This method checks for collisions between drones and all other drones in the environment.
        By using interpolation and all the drone's known velocities and positions and their progression a universal and continuous time model is created to compare each drone's position at a specific moment in time.

        :param resolution: The resolution determines the size of the steps in time on which the collision detection should be performed between drones. A lower resolution implies more collision checks will be performed.
        :return: A list of all collisions between drones and other drones at any moment in time
        """
        collisions_drones: list[tuple[int, int]] = []
        t_max: float = max(float(drone.path.t[-2]) for drone in self.drones) # Moment in time in which the last drone passes its last control point (Goal excluded)
        t_min: float = min(float(drone.path.t[1]) for drone in self.drones) # Moment in time in which the first drone passes its first control point (Start excluded)
        t_samples = np.arange(t_min, t_max, resolution) # Create an even distribution along the time-axis

        for t in t_samples: # For every moment in time
            positions = [
                (drone.path.x(float(t)), drone.path.y(float(t)), drone.radius) for drone in self.drones # Get all the drones current position at the currently reviewed moment in time
            ]

            for i in range(len(positions)):
                for j in range(i + 1, len(positions)): # Check collisions between drones
                    x1, y1, r1 = positions[i]
                    x2, y2, r2 = positions[j]
                    if _collision_raw(x1, y1, r1, x2, y2, r2):
                        collisions_drones.append((x1, y1))
        return collisions_drones

def _collision_raw(x1: float, y1: float, r1: float, x2: float, y2: float, r2: float) -> bool:
    """
    This method calculates collisions between objects using Euclidean distance.

    :param x1: X-coordinate object 1
    :param y1: Y-coordinate object 1
    :param r1: Radius object 1
    :param x2: X-coordinate object 2
    :param y2: Y-coordinate object 2
    :param r2: Radius object 2
    :return: True if collision detected, False otherwise
    """
    dx = x2 - x1
    dy = y2 - y1
    dist_sq = dx * dx + dy * dy
    return dist_sq < (r2 + r1)**2