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

    Attributes:
        DEBUG (bool): Enables debug mode if set to True.
        CAMERAS_UPDATE_RATE (int): Update interval in seconds for camera and matrix data.
        ENCODING (str): Encoding format used for data serialization (default: 'utf-8').
        ADDRESS_CALIBRATION (str): Network address of the calibration component.
        ADDRESS_SKELETON (str): Network address of the skeleton component.
        ADDRESS_VISUALIZATION (str): Network address of the visualization component (optional).
        HOSTNAME (str): Hostname where this application is served.
        PORT (int): Port number for serving this application.
    """
    DEBUG: bool = False

    # Update rate of cameras and corresponding matrices
    CAMERAS_UPDATE_RATE: int = 300  # 300s -> 5min

    # Data encoding
    ENCODING: str = 'utf-8'

    # Addresses of dependent components
    ADDRESS_CALIBRATION: str = 'localhost:5555'
    ADDRESS_SKELETON: str = 'localhost:5556'
    ADDRESS_VISUALIZATION: str = 'localhost:5558'  # Not used since data is published via ZeroMQ

    # Address this software is running under
    HOSTNAME: str = 'localhost'
    PORT: int = 5557

@lru_cache # Only create the first instance and return the cached instance otherwise
def get_settings() -> Settings:
    """
    Retrieve a singleton instance of the Settings object.

    Uses LRU caching to ensure that the same instance is reused across the application.
    This is both efficient and ensures consistency across imports.

    :return: Settings: The application settings instance.
    """
    return Settings()