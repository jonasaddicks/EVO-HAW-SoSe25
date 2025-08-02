import colorsys

import matplotlib
import matplotlib.pyplot as plt

from DroneSwarmPathOpti.config import get_settings

matplotlib.use('TkAgg')
import numpy as np
from matplotlib import cm
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle
from matplotlib.widgets import Slider
from scipy.interpolate import CubicSpline, interp1d

from DroneSwarmPathOpti.simulation import Environment

settings = get_settings()

def plot_environment(environment: Environment):
    fig, ax = plt.subplots()
    plt.title("Map")

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

    t_max: float = 0
    for i, drone in enumerate(environment.drones):
        spline = drone.path
        if spline.t[-1] > t_max:
            t_max = float(spline.t[-1])
        t_vals = np.linspace(spline.t[0], spline.t[-1], 200)

        x_vals = spline.x(t_vals)
        y_vals = spline.y(t_vals)

        # Velocity interpolation
        v_vals = interp1d(spline.t, [p[2] for p in spline.raw_path], kind='linear', fill_value='extrapolate')(t_vals)
        v_norm = (v_vals - 0.1) / (settings.DRONE_MAX_SPEED - 0.1 + 1e-9) # Normalizing

        # Base color for the path
        base_rgb = cm.get_cmap("brg", len(environment.drones))(i)[:3]
        h, l, s = colorsys.rgb_to_hls(*base_rgb)

        # Adjust every point's brightness according to velocity
        colors = [colorsys.hls_to_rgb(h, l * (0.2 + 0.8 * vn), s) for vn in v_norm]

        # build line segments
        points = np.array([x_vals, y_vals]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        lc = LineCollection(segments, colors=colors[:-1], linewidth=2)
        ax.add_collection(lc)

    # Draw collisions
    for collision in environment.get_collisions_drones():
        x, y = collision
        ax.plot(x, y, color='#ff6f00', marker='o', markersize=4, linestyle='None')
    for collision in environment.get_collisions_obstacles():
        x, y = collision
        ax.plot(x, y, color='#e61d12', marker='o', markersize=3, linestyle='None')

    for drone in environment.drones:
        for control_point in drone.path.raw_path[1:-1]:
            ax.plot(control_point[0], control_point[1], color='#e612d8', marker='x', markersize=6, linestyle='None')

    ax_slider = plt.axes((0.15, 0.02, 0.7, 0.04))
    slider = Slider(ax_slider, 't', 0, t_max, valinit=0, valstep=t_max / 200)

    drone_circles: list[Circle] = []
    for drone in environment.drones:
        circle = Circle(environment.start.position, drone.radius, color='pink', alpha=0.6)
        drone_circles.append(circle)
        ax.add_patch(circle)

    def update(val):
        t_current = slider.val
        for _circle, _drone in zip(drone_circles, environment.drones):
            if t_current < _drone.path.t[-1]:
                new_pos: tuple[float, float] = (_drone.path.x(t_current), _drone.path.y(t_current))
                _circle._center = new_pos
            else:
                new_pos: tuple[float, float] = (_drone.path.x(_drone.path.t[-1]), _drone.path.y(_drone.path.t[-1]))
                _circle._center = new_pos
        fig.canvas.draw_idle()

    slider.on_changed(update)

    plt.grid(True)
    ax.set_aspect('equal')
    ax.set_xlim(0, environment.bounds[0])
    ax.set_ylim(0, environment.bounds[1])
    plt.show()
