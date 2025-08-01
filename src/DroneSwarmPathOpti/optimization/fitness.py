from .particle import Particle
from DroneSwarmPathOpti.simulation import CubicBSpline, Environment
from ..config import get_settings

settings = get_settings()


def calculate_fitness(particle: Particle, environment: Environment) -> float:
    energy_usage: float = 0
    time_usage: float = 0
    map_breaches: int = 0

    splines: list[CubicBSpline] = [
        CubicBSpline(
            [(environment.start.position[0], environment.start.position[1], 1.0)]
            + path.control_points
            + [(environment.goal.position[0], environment.goal.position[1], 1.0)]
        ) for path in particle.particle_position]

    for drone, spline in zip(environment.drones, splines): # Assign a path to each drone in the environment.
        drone.path = spline
        energy_usage += spline.calculate_energy_usage()
        time_usage += spline.calculate_time_usage()
        for x, y, _ in spline.raw_path:
            if 0 > x or x > environment.bounds[0] or 0 > y or y > environment.bounds[1]: # Point of spline outside map bounds
                map_breaches += 1
    collisions: int = environment.number_of_collisions()

    return (
            settings.FITNESS_WEIGHT_TIME * time_usage
            +
            settings.FITNESS_WEIGHT_ENERGY * energy_usage
            +
            settings.FITNESS_WEIGHT_COLLISIONS * collisions
            +
            settings.FITNESS_WEIGHT_BREACHES * map_breaches
    )
    # TODO implement a version considering distance to goal (without the goal already being part of the spline)