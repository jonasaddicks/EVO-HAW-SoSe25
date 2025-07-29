import networkx as nx


def _grid_to_graph(grid: list[list[int]]) -> nx.Graph:
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
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def traverse(
        grid_data: list[list[int]],
        start: tuple[int, int],
        goal: tuple[int, int]
) -> list[tuple[int, int]]:
    grid = _grid_to_graph(grid_data)
    try:
        return nx.astar_path(grid, start, goal, heuristic=_heuristic)
    except nx.exception.NetworkXNoPath:
        return []