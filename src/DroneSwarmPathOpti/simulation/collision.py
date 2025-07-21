from DroneSwarmPathOpti.simulation.environment import MapObject


def collision(map_object_a: MapObject, map_object_b: MapObject) -> bool:
    x1, y1 = map_object_a.position
    r1 = map_object_a.radius
    x2, y2 = map_object_b.position
    r2 = map_object_b.radius

    return (x2 - x1) ** 2 + (y2 - y1) ** 2 < (r1 + r2) ** 2