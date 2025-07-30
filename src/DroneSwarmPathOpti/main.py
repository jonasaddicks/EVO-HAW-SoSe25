from DroneSwarmPathOpti.config import get_settings
from DroneSwarmPathOpti.simulation import Environment, Drone
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
    plot_environment(environment)

if __name__ == '__main__':
    main()