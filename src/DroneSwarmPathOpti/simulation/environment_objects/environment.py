import random

from .map_object import MapObject
from .map_object import collision

from ..environment_utils import traverse

class Obstacle(MapObject):

    def __init__(self, position: tuple[int, int], base_radius: float):
        super().__init__(position, random.uniform(base_radius - base_radius*0.5, base_radius*1.5))


class Environment:
    bounds: tuple[int, int]
    obstacles: list[Obstacle]
    start: MapObject | None
    goal: MapObject | None

    traversable:bool # True if there has to be at least one path from start to finish
    validation_path: list[tuple[int, int]] | None # None if not traversable

    def __init__(self,
                 bounds: tuple[int, int],
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
        self.obstacles = []

    def generate_obstacles(self,
                           obstacles: int,
                           base_radius: float
                           ) -> bool:

        (x_max, y_max) = self.bounds
        tries_traversable: int = 0
        while True:
            obstacles_list: list[Obstacle] = []
            for _ in range(obstacles):
                obstacle: Obstacle
                tries_obstacle: int = 0

                while True:
                    x = random.randint(0, x_max)
                    y = random.randint(0, y_max)
                    obstacle = Obstacle((x, y), base_radius)
                    if (
                            (self.start is None or self.goal is None)
                            or
                            not (collision(obstacle, self.start) or collision(obstacle, self.goal))):
                        break  # Exit loop if generated object collides neither with start nor goal
                    else:
                        tries_obstacle += 1
                        print(f'obstacle clipping with start or goal in try {tries_obstacle} - placing again')
                        if tries_obstacle >= 10:
                            print(f'exceeded number of tries for placing an obstacle, obstacles remain empty')
                            return False # Exceeded maximum number of tries to generate an obstacle -> obstacles probably too big
                obstacles_list.append(obstacle)
            self.obstacles = obstacles_list

            if self.traversable and self.start is not None and self.goal is not None:
                path: list[tuple[int, int]] = self._validate_map() # Check if map is traversable from start to goal
                if len(path) > 0: # Map is traversable
                    self.validation_path = path
                    break
                else:
                    tries_traversable += 1
                    print(f'failed to traverse map in try {tries_traversable} - regenerating obstacles')
                    if tries_traversable >= 10:
                        self.validation_path = [] # Map should've been traversable but the algorithm was not able to find a feasible path with the given amount of tries
                        print('failed to find a valid path - max number of tries exceeded, path is empty')
                        return False # Exceeded maximum number of tries to traverse the map -> probably too many obstacles
            else: # Map must not be traversable or start/goal is none
                self.validation_path = None
                break
        return True

    def _validate_map(self) -> list[tuple[int, int]]:
        width, height = self.bounds
        grid_data = [[0 for _ in range(width)] for _ in range(height)]

        for obstacle in self.obstacles:
            ox, oy = obstacle.position
            r = int(obstacle.radius)
            for y in range(max(0, oy - r), min(height, oy + r)):
                for x in range(max(0, ox - r), min(width, ox + r)):
                    if (x - ox) ** 2 + (y - oy) ** 2 <= r ** 2:
                        grid_data[y][x] = 1

        start = self.start.position
        goal = self.goal.position

        path = traverse(grid_data, start, goal)
        return path