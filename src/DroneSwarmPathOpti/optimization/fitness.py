from .particle import Particle
from DroneSwarmPathOpti.simulation import CubicBSpline, Environment
from ..config import get_settings

settings = get_settings()


def calculate_fitness(particle: Particle, environment: Environment) -> float:
    energy_usage: float = 0
    time_usage: float = 0

    splines: list[CubicBSpline] = [CubicBSpline(path.control_points) for path in particle.particle_position]
    for drone, spline in zip(environment.drones, splines): # Assign a path to each drone in the environment.
        drone.path = spline
        energy_usage += spline.calculate_energy_usage()
        time_usage += spline.calculate_time_usage()
    collisions: int = environment.number_of_collisions()

    return (
            settings.FITNESS_WEIGHT_TIME * time_usage
            +
            settings.FITNESS_WEIGHT_ENERGY * energy_usage
            +
            settings.FITNESS_WEIGHT_COLLISIONS * collisions
    )
    # TODO implement a version considering distance to goal (without the goal already being part of the spline)