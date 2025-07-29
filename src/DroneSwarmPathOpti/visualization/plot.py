import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from DroneSwarmPathOpti.simulation import Environment


def plot_environment(environment: Environment):
    fig, ax = plt.subplots()
    ax.set_xlim(0, environment.bounds[0])
    ax.set_ylim(0, environment.bounds[1])
    ax.set_aspect('equal')

    # draw obstacles
    for obstacle in environment.obstacles:
        circle = Circle(obstacle.position, obstacle.radius, color='black', alpha=0.5)
        ax.add_patch(circle)

    if environment.start:
        circle = Circle(environment.start.position, environment.start.radius, color='green', alpha=0.3)
        ax.add_patch(circle)

    if environment.goal:
        circle = Circle(environment.goal.position, environment.goal.radius, color='blue', alpha=0.3)
        ax.add_patch(circle)

    if environment.traversable and len(environment.validation_path) > 0:
        x_vals, y_vals = zip(*environment.validation_path)
        ax.plot(x_vals, y_vals, color='red', linewidth=2, label="Path")

    plt.grid(True)
    plt.title("Map")
    plt.show()
