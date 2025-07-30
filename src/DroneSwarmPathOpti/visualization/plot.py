import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from DroneSwarmPathOpti.simulation import Environment


def plot_environment(environment: Environment):
    fig, ax = plt.subplots()
    ax.set_xlim(0, environment.bounds[0])
    ax.set_ylim(0, environment.bounds[1])
    ax.set_aspect('equal')

    # Draw obstacles
    for obstacle in environment.obstacles:
        circle = Circle(obstacle.position, obstacle.radius, color='black', alpha=0.5)
        ax.add_patch(circle)

    # Draw the start
    if environment.start:
        circle = Circle(environment.start.position, environment.start.radius, color='green', alpha=0.3)
        ax.add_patch(circle)

    # Draw the goal
    if environment.goal:
        circle = Circle(environment.goal.position, environment.goal.radius, color='blue', alpha=0.3)
        ax.add_patch(circle)

    # Draw a possible route from start to goal (using a*)
    if environment.traversable and len(environment.validation_path) > 0:
        x_vals, y_vals = zip(*environment.validation_path)
        ax.plot(x_vals, y_vals, color='red', linewidth=2, label="Path")

    """
    # Draw a temporary test spline
    spline = get_test_spline()
    spline_x = spline.x
    spline_y = spline.y
    spline_t = spline.t
    t_range = (spline_t[0], spline_t[-1])

    t_vals = np.linspace(t_range[0], t_range[1], 200)
    x_vals = spline_x(t_vals)
    y_vals = spline_y(t_vals)
    ax.plot(x_vals, y_vals, linewidth=2)
    """

    plt.grid(True)
    plt.title("Map")
    plt.show()
