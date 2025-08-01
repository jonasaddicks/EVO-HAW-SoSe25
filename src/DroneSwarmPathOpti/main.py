from DroneSwarmPathOpti.config import get_settings
from DroneSwarmPathOpti.optimization.fitness import calculate_fitness
from DroneSwarmPathOpti.optimization.pso import PSO
from DroneSwarmPathOpti.simulation import Environment, Drone, CubicBSpline
from DroneSwarmPathOpti.visualization.plot import plot_environment

settings = get_settings()

def main():
    drones: list[Drone] = [
        Drone(None,
              (settings.START_X,
               settings.START_Y),
              settings.DRONE_RADIUS)
        for _ in range(settings.NUMBER_DRONES)
    ]

    environment: Environment = Environment(
        (settings.ENVIRONMENT_SIZE_X, settings.ENVIRONMENT_SIZE_Y),
        drones,
        settings.ENVIRONMENT_TRAVERSABLE,
        (settings.START_X, settings.START_Y),
        settings.START_RADIUS,
        (settings.GOAL_X, settings.GOAL_Y),
        settings.GOAL_RADIUS
    )
    environment.generate_obstacles(settings.NUMBER_OBSTACLES, settings.AVG_SIZE_OBSTACLE)

    pso: PSO = PSO(calculate_fitness, environment, settings.PSO_PARTICLES, settings.PSO_ITERATIONS)
    solution = pso.optimize()

    for drone, path in zip(drones, solution[0]):
        spline: CubicBSpline = CubicBSpline(
            [(environment.start.position[0], environment.start.position[1], 1.0)]
            + path.control_points
            + [(environment.goal.position[0], environment.goal.position[1], 1.0)]
        )
        drone.path = spline
    plot_environment(environment)

if __name__ == '__main__':
    main()