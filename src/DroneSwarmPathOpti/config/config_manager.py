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
    SEED_ENVIRONMENT: int = 13
    SEED_PARTICLE: int = 42

    # DRONE PARAMETERS
    NUMBER_DRONES: int = 5
    DRONE_RADIUS: float = 1.0
    DRONE_MAX_SPEED: float = 10.0
    INITIAL_CONTROL_POINTS: int = 5

    # ENVIRONMENT PARAMETERS
    ENVIRONMENT_SIZE_X: int = 100
    ENVIRONMENT_SIZE_Y: int = 100

    ENVIRONMENT_TRAVERSABLE: bool = True
    NUMBER_OBSTACLES: int = 10
    AVG_SIZE_OBSTACLE: int = 20

    START_X: int = 10
    START_Y: int = 10
    START_RADIUS: int = 10

    GOAL_X: int = 90
    GOAL_Y: int = 90
    GOAL_RADIUS: int = 10

    # PARTICLE SWARM OPTIMIZATION PARAMETERS
    PSO_PARTICLES: int = 30
    PSO_ITERATIONS: int = 200

    PSO_MAX_INITIAL_VELOCITY_X: float = 1.0
    PSO_MAX_INITIAL_VELOCITY_Y: float = 1.0
    PSO_MAX_INITIAL_VELOCITY_DRONE_VELOCITY: float = 0.3
    PSO_INITIAL_POSITION_BOUNDS: float = 30
    PSO_INITIAL_DISTANCE_PATHS: float = 10

    PSO_MAX_VELOCITY_X: float = 5.0
    PSO_MAX_VELOCITY_Y: float = 5.0
    PSO_MAX_VELOCITY_DRONE_VELOCITY: float = 2.0
    PSO_VELOCITY_DAMPING: float = 0.5

    PSO_FLUSH_SHARE: float = 0.03
    PSO_FLUSH_WHEN: float = 0.5

    PSO_DECREASE_MAX_VELOCITY_WHEN: float = 0.5
    PSO_DECREASE_MAX_VELOCITY_GOAL: float = 3.0
    PSO_DECREASE_INITIAL_VELOCITY_WHEN: float = 0.7
    PSO_DECREASE_INITIAL_VELOCITY_GOAL: float = 1.5

    PSO_WEIGHT_PERSONAL_POSITION: float = 1.0
    PSO_WEIGHT_PERSONAL_BEST: float = 1.0
    PSO_WEIGHT_GLOBAL_BEST: float = 1.0

    PSO_INCREASE_WEIGHT_GLOBAL_WHEN: float = 0.8
    PSO_INCREASE_WEIGHT_GLOBAL_GOAL: float = 0.9
    PSO_DECREASE_WEIGHT_PERSONAL_WHEN: float = 0.75
    PSO_DECREASE_WEIGHT_PERSONAL_GOAL: float = 0.5

    FITNESS_WEIGHT_ENERGY: float = 1.0
    FITNESS_WEIGHT_TIME: float = 1.0
    FITNESS_WEIGHT_COLLISIONS_OBSTACLES: float = 1.0
    FITNESS_WEIGHT_COLLISIONS_DRONES: float = 1.0

@lru_cache # Only create the first instance and return the cached instance otherwise
def get_settings() -> Settings:
    """
    Retrieve a singleton instance of the Settings object.

    Uses LRU caching to ensure that the same instance is reused across the application.
    This is both efficient and ensures consistency across imports.

    :return: Settings: The application settings instance.
    """
    return Settings()