from ..environment_utils import CubicBSpline
from .map_object import MapObject


class Drone(MapObject):
    """
    This class represents a drone in an environment.
    """

    path: CubicBSpline | None

    def __init__(
            self,
            path: CubicBSpline | None,
            position: tuple[int, int],
            radius: float):
        super().__init__(position, radius)
        self.path = path