from ..environment_utils import CubicBSpline
from .map_object import MapObject


class Drone(MapObject):

    path: CubicBSpline | None

    def __init__(
            self,
            path: CubicBSpline | None,
            position: tuple[int, int],
            radius: float):
        super().__init__(position, radius)
        self.path = path