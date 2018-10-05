import unittest
import sphero_tracker
from geometry_msgs.msg import Point

class MyTestCase(unittest.TestCase):

    def test_something(self):
        t = sphero_tracker.SpheroTracker()

        pt = t.filter('red', Point(10,10,0))


        self.assertIsNotNone(pt)


if __name__ == '__main__':
    unittest.main()
