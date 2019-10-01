from ..rosweld.point import Point
from geometry_msgs.msg import Point as GeoPoint

class TestPoint(object):
    def test_init(self):
        p = Point(1,2,3)
        assert p.x == 1 and p.y == 2 and p.z == 3
        assert isinstance(p, Point) and isinstance(p, GeoPoint)

    def test_operators(self):
        p = Point(0,0,0)
        p += Point(1,1,1)
        assert p.x == 1 and p.y == 1 and p.z == 1
        p -= Point(1,1,1)
        assert p.x == 0 and p.y == 0 and p.z == 0
        p += Point(2,2,2)
        p /= 2
        assert p.x == 1 and p.y == 1 and p.z == 1
        p = p + Point(1,0,0)
        assert p.x == 2 and p.y == 1 and p.z == 1
        p = Point(5,5,5)
        assert p.x == 5 and p.y == 5 and p.z == 5
        p = p / 5
        assert p.x == 1 and p.y == 1 and p.z == 1
