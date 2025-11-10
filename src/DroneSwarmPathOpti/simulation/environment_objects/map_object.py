class MapObject:
    """
    This class represents the root class of any object in the environment.
    Any further specification of an object existing in an environment inherits from this class.
    """

    position: tuple[int, int]
    radius: float

    def __init__(self, position: tuple[int, int], radius: float):
        self.position = position
        self.radius = radius

def collision_objects(map_object_a: MapObject, map_object_b: MapObject) -> bool:
    """
    This method checks whether two MapObjects collide with each other or not. If they collide, returns True.

    :param map_object_a: First MapObject
    :param map_object_b: Second MapObject
    :return: True if collision, False otherwise
    """
    x1, y1 = map_object_a.position
    r1 = map_object_a.radius
    x2, y2 = map_object_b.position
    r2 = map_object_b.radius

    return (x2 - x1) ** 2 + (y2 - y1) ** 2 < (r1 + r2) ** 2