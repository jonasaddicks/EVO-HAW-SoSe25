from functools import lru_cache

from pydantic_settings import BaseSettings
from dotenv import load_dotenv, find_dotenv

env_file = find_dotenv(".env.public", raise_error_if_not_found=True) # Locate the .env.public file and raise an error if it is not found.
load_dotenv(override=True, dotenv_path=env_file) # Load environment variables from the located file, overriding any existing ones.

class Settings(BaseSettings):
    """
    Configuration class for application-wide settings.

    Settings are loaded from environment variables (loaded beforehand via dotenv).
    Pydantic handles type validation and parsing.
    """
    DEBUG: bool = False

    # DRONE PARAMETERS
    NUMBER_DRONES: int = 5 # Number of drones in an environment
    DRONE_RADIUS: float = 1.0 # Size of a drone
    DRONE_MAX_SPEED: float = 10.0 # Maximum drone speed
    INITIAL_CONTROL_POINTS: int = 5 # Amount of points in a single drone path from start to goal

    # ENVIRONMENT PARAMETERS
    ENVIRONMENT_SIZE_X: int = 100 # Width of the environment
    ENVIRONMENT_SIZE_Y: int = 100 # Height of the environment

    ENVIRONMENT_TRAVERSABLE: bool = True # Forces at least one path without any collisions from start to goal (NOTE: depending on environment size and amount of obstacles the calculation power needed can be exceedingly high.)
    NUMBER_OBSTACLES: int = 10 # Number of obstacles in the environment
    AVG_SIZE_OBSTACLE: int = 20 # Average size of all the obstacles

    START_X: int = 10 # Starting point X-coordinate
    START_Y: int = 10 # Starting point Y-coordinate
    START_RADIUS: int = 10 # Starting point radius

    GOAL_X: int = 90 # Goal point X-coordinate
    GOAL_Y: int = 90 # Goal point Y-coordinate
    GOAL_RADIUS: int = 10 # Goal point radius

    # PARTICLE SWARM OPTIMIZATION PARAMETERS
    PSO_PARTICLES: int = 30 # Number of particles to explore the solution space
    PSO_ITERATIONS: int = 200 # Number of iterations the particle swarm optimization will perform

    PSO_MAX_INITIAL_VELOCITY_X: float = 1.0 # Max velocity of particle (X) when initializing for the first time
    PSO_MAX_INITIAL_VELOCITY_Y: float = 1.0 # Max velocity of particle (Y) when initializing for the first time
    PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY: float = 0.3 # Max drone velocity when initializing for the first time
    PSO_INITIAL_POSITION_BOUNDS: float = 30 # Area around an initial point when generating in which the actual point generates for the first time
    PSO_INITIAL_DISTANCE_PATHS: float = 10 # Probable distance between different drone paths when initializing for the first time

    PSO_MAX_VELOCITY_X: float = 5.0 # Max velocity of a particle (X)
    PSO_MAX_VELOCITY_Y: float = 5.0 # Max velocity of a particle (Y)
    PSO_MAX_VELOCITY_DRONE_VELOCITY: float = 2.0 # Max velocity of a particle (drone velocity)
    PSO_VELOCITY_DAMPING: float = 0.5 # Scalar which a particle is scaled with when violating environment bounds and bouncing back

    PSO_FLUSH_SHARE: float = 0.03 # Portion of particles to be flushed each generation
    PSO_FLUSH_WHEN: float = 0.5 # After how many generations will the flush occur for the first time (depending on the max amount of iterations)

    PSO_DECREASE_MAX_VELOCITY_WHEN: float = 0.5 # After how many generations will the max velocity begin to adapt (depending on the max amount of iterations)
    PSO_DECREASE_MAX_VELOCITY_GOAL: float = 3.0 # Max velocity value to gradually be approached through the generations
    PSO_DECREASE_INITIAL_VELOCITY_WHEN: float = 0.7 # After how many generations will the initial velocity begin to adapt (depending on the max amount of iterations)
    PSO_DECREASE_INITIAL_VELOCITY_GOAL: float = 1.5 # Max initial velocity value to gradually be approached through the generations

    PSO_WEIGHT_PERSONAL_POSITION: float = 1.0 # Weight the current personal position of a particle
    PSO_WEIGHT_PERSONAL_BEST: float = 1.0 # Weight the current best personal position of a particle
    PSO_WEIGHT_GLOBAL_BEST: float = 1.0 # Weight the current best global position of all particles

    PSO_INCREASE_WEIGHT_GLOBAL_WHEN: float = 0.8 # After how many generations will the global weight begin to adapt (depending on the max amount of iterations)
    PSO_INCREASE_WEIGHT_GLOBAL_GOAL: float = 0.9 # Global weight value to gradually be approached through the generations
    PSO_DECREASE_WEIGHT_PERSONAL_WHEN: float = 0.75 # After how many generations will the personal weight begin to adapt (depending on the max amount of iterations)
    PSO_DECREASE_WEIGHT_PERSONAL_GOAL: float = 0.5 # Personal weight value to gradually be approached through the generations

    FITNESS_WEIGHT_ENERGY: float = 1.0 # How important is energy usage
    FITNESS_WEIGHT_TIME: float = 1.0 # How important is time usage
    FITNESS_WEIGHT_COLLISIONS_OBSTACLES: float = 1.0 # How important is obstacle collision prevention
    FITNESS_WEIGHT_COLLISIONS_DRONES: float = 1.0 # How important is drone collision prevention

@lru_cache # Only create the first instance and return the cached instance otherwise
def get_settings() -> Settings:
    """
    Retrieve a singleton instance of the Settings object.

    Uses LRU caching to ensure that the same instance is reused across the application.
    This is both efficient and ensures consistency across imports.

    :return: Settings: The application settings instance.
    """
    return Settings()