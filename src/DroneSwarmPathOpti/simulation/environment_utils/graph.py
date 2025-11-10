import networkx as nx


def _grid_to_graph(grid: list[list[int]]) -> nx.Graph:
    """
    This method takes a two-dimensional grid and converts it into a networkx graph.

    :param grid: Two-dimensional grid in which every integer but a '1' represents a node to be connected to all its neighbors. A '1' will be skipped.
    :return:
    """
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
                        nx_grid.add_edge((x, y), (nx_, ny_), weight=1)

    return nx_grid

def _heuristic(a, b):
    """
    This method calculates the heuristic between two nodes.

    :param a: Node a
    :param b: Node b
    :return: The heuristic as an absolute value.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def traverse(
        grid_data: list[list[int]],
        start: tuple[int, int],
        goal: tuple[int, int]
) -> list[tuple[int, int]]:
    """
    This method traverses a grid of nodes starting from start to goal.

    :param grid_data: The graph to be traversed.
    :param start: Starting node
    :param goal: Goal node
    :return: A list of nodes - represented as tuples with their respective x and y coordinates within the grid - containing a possible path from start to goal. If no path was found an empty list will be returned.
    """
    grid = _grid_to_graph(grid_data)
    try:
        return nx.astar_path(grid, start, goal, heuristic=_heuristic)
    except nx.exception.NetworkXNoPath:
        return []