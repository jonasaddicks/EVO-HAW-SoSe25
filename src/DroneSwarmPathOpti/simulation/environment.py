import random
import networkx as nx

class MapObject:

    position: tuple[int, int]
    radius: float

    def __init__(self, position: tuple[int, int], radius: float):
        self.position = position
        self.radius = radius


class Obstacle(MapObject):

    def __init__(self, position: tuple[int, int], base_radius: float):
        super().__init__(position, random.uniform(base_radius - base_radius*0.5, base_radius*1.5))


class Environment:
    bounds: tuple[int, int]
    obstacles: list[Obstacle]
    start: MapObject | None
    goal: MapObject | None

    traversable:bool # True if there has to be at least one path from start to finish
    validation_path: list[tuple[int, int]] | None

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
                           ):

        from DroneSwarmPathOpti.simulation import collision

        (x_max, y_max) = self.bounds
        while True:
            obstacles_list: list[Obstacle] = []
            for _ in range(obstacles):
                obstacle: Obstacle

                while True:
                    x = random.randint(0, x_max)
                    y = random.randint(0, y_max)
                    obstacle = Obstacle((x, y), base_radius)
                    if (
                            (self.start is None or self.goal is None)
                            or
                            not (collision(obstacle, self.start) or collision(obstacle, self.goal))):
                        break  # Exit loop if generated object collides neither with start nor goal
                obstacles_list.append(obstacle)
            self.obstacles = obstacles_list

            if self.start is not None and self.goal is not None and self.traversable:
                path: list[tuple[int, int]] = self._validate_map() # Check if map is traversable from start to goal
                if len(path) > 0: # Map is traversable
                    self.validation_path = path
                    break
            else: # Map must not be traversable or start/goal is none
                self.validation_path = None
                break

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

        grid = _grid_to_graph(grid_data)
        start = self.start.position
        goal = self.goal.position

        path = nx.astar_path(grid, start, goal, heuristic=_heuristic)
        return path


def _grid_to_graph(grid):
    nx_grid = nx.Graph()
    height = len(grid)
    width = len(grid[0])

    for y in range(height):
        for x in range(width):
            if grid[y][x] == 1:
                continue  # skip obstacle

            nx_grid.add_node((x, y))

            # connect neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx_, ny_ = x + dx, y + dy
                if 0 <= nx_ < width and 0 <= ny_ < height:
                    if grid[ny_][nx_] == 0:
                        nx_grid.add_edge((x, y), (nx_, ny_))

    return nx_grid

def _heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])