import colorsys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.patches import Circle
from scipy.interpolate import CubicSpline

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

    cmap = cm.get_cmap('brg', len(environment.drones))
    for i, drone in enumerate(environment.drones):
        spline = drone.path
        t_vals = np.linspace(spline.t[0], spline.t[-1], 200)

        x_vals = spline.x(t_vals)
        y_vals = spline.y(t_vals)

        # Geschwindigkeit interpolieren
        v_vals = CubicSpline(spline.t, [p[2] for p in spline.raw_path])(t_vals)
        v_norm = (v_vals - v_vals.min()) / (v_vals.max() - v_vals.min() + 1e-9)

        # Grundfarbe aus Colormap
        base_rgb = cm.get_cmap("brg", len(environment.drones))(i)[:3]  # RGB (ohne Alpha)
        h, l, s = colorsys.rgb_to_hls(*base_rgb)

        # Für jeden Punkt neue RGB-Farbe mit variierender Lightness
        colors = [colorsys.hls_to_rgb(h, l * (0.3 + 0.7 * vn), s) for vn in v_norm]

        # Liniensegmente bilden
        points = np.array([x_vals, y_vals]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Zeichnen
        lc = LineCollection(segments, colors=colors[:-1], linewidth=2)
        ax.add_collection(lc)

    # Draw collisions
    for collision in environment.get_collisions():
        x, y = collision
        ax.plot(x, y, color='#e61d12', marker='o', markersize=3, linestyle='None')

    for drone in environment.drones:
        for control_point in drone.path.raw_path[1:-1]:
            ax.plot(control_point[0], control_point[1], color='#e612d8', marker='x', markersize=6, linestyle='None')

    plt.grid(True)
    plt.title("Map")
    plt.show()
