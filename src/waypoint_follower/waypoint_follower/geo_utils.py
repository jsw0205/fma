import math

EARTH_RADIUS_M = 6378137.0


def latlon_to_mercator(lat, lon):
    """Web Mercator projection (matches the validated Windows GPS-driving
    code this was ported from) -- globally consistent scale, locally
    accurate enough for waypoint following over short distances."""
    x = EARTH_RADIUS_M * math.radians(lon)
    y = EARTH_RADIUS_M * math.log(math.tan((90.0 + lat) * math.pi / 360.0))
    return x, y


class LocalFrame:
    """Re-centers Web Mercator coordinates on an origin lat/lon so
    RViz markers stay near (0, 0) instead of ~10,000,000 m out."""

    def __init__(self, origin_lat, origin_lon):
        self.origin_x, self.origin_y = latlon_to_mercator(origin_lat, origin_lon)

    def to_enu(self, lat, lon):
        x, y = latlon_to_mercator(lat, lon)
        return x - self.origin_x, y - self.origin_y


def distance_m(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)
