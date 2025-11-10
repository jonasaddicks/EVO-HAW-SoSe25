from .particle import DronePath
from DroneSwarmPathOpti.simulation import CubicBSpline, Environment
from ..config import get_settings

settings = get_settings()


def calculate_fitness(particle_position: list[DronePath], environment: Environment) -> float:
    """
    Calculates the fitness value of a particle in a given environment.

    :param particle_position: A particle which represents a full approach to a solution to the given environment.
    :param environment: The environment in which the particles exist.
    :return: A fitness value of a given particle in a given environment as a float.
    """
    energy_usage: float = 0
    time_usage: float = 0

    splines: list[CubicBSpline] = [
        CubicBSpline(
            [(environment.start.position[0], environment.start.position[1], 1.0)]
            + path.control_points
            + [(environment.goal.position[0], environment.goal.position[1], 1.0)]
        ) for path in particle_position] # Build splines out of the provided drone paths by adding the environment's start and goal points to each drone's path

    for drone, spline in zip(environment.drones, splines): # Assign a path to each drone in the environment. (Link the environment's drones to the provided paths)
        drone.path = spline
        energy_usage += spline.calculate_energy_usage()
        time_usage += spline.calculate_time_usage()

    number_collisions_obstacles: int = len(environment.get_collisions_obstacles())
    number_collisions_drones: int = len(environment.get_collisions_drones())

    return (
            settings.FITNESS_WEIGHT_TIME * time_usage
            +
            settings.FITNESS_WEIGHT_ENERGY * energy_usage
            +
            settings.FITNESS_WEIGHT_COLLISIONS_OBSTACLES * number_collisions_obstacles
            +
            settings.FITNESS_WEIGHT_COLLISIONS_DRONES * number_collisions_drones
    )