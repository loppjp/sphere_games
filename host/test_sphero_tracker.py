import unittest
import sphero_tracker
from geometry_msgs.msg import Point

class MyTestCase(unittest.TestCase):

    def test_filter_initial(self):
        t = sphero_tracker.SpheroTracker()
        t.running_average = {}
        expected = Point(10,10,0)

        pt = t.filter('red', expected)


        self.assertEquals(pt, expected)


    def test_filter_simple_01(self):
        t = sphero_tracker.SpheroTracker()
        t.running_average = {}

        input_list = \
            [Point(10, 10, 0),
             Point(10, 10, 0),
             Point(10, 10, 0),
             Point(10, 10, 0),
             Point(10, 10, 0),
             ]

        expected = Point(10,10,0)

        for pt in input_list:
            result = t.filter('red', pt)


        self.assertEquals(result, expected)

    def test_filter_simple_01(self):
        '''
        Should still give last point at 10,10,0
        :return:
        '''
        t = sphero_tracker.SpheroTracker()
        t.running_average = {}

        input_list = \
            [Point(10, 10, 0),
             Point(10, 10, 0),
             Point(12, 10, 0),
             Point(9, 10, 0),
             Point(10, 10, 0),
             ]

        expected = Point(10,10,0)

        for pt in input_list:
            result = t.filter('red', pt)


        self.assertEquals(result, expected)

    def test_filter_simple_02(self):
        '''
        Should still give last point at 10,10,0
        :return:
        '''
        t = sphero_tracker.SpheroTracker()
        t.running_average = {}

        input_list = \
            [Point(10, 10, 0),
             Point(10, 10, 0),
             Point(12, 10, 0),
             Point(9, 10, 0),
             Point(12, 10, 0),
             ]

        expected = Point(12,10,0)

        for pt in input_list:
            result = t.filter('red', pt)


        self.assertEquals(result, expected)


    def test_filter_simple_03(self):
        '''
        Should still give last point at 10,10,0
        :return:
        '''
        t = sphero_tracker.SpheroTracker()
        t.running_average = {}

        input_list = \
            [Point(10, 10, 0),
             Point(10, 10, 0),
             Point(12, 10, 0),
             Point(9, 10, 0),
             Point(80, 90, 0),
             ]

        expected = Point(9,10,0)

        for pt in input_list:
            result = t.filter('red', pt)


        self.assertEquals(result, expected)

    def test_filter_simple_04(self):
        '''
        Should still give last point at 10,10,0
        :return:
        '''
        t = sphero_tracker.SpheroTracker()
        t.running_average = {}

        input_list = \
            [None
             ]

        expected = None

        for pt in input_list:
            result = t.filter('red', pt)


        self.assertEquals(result, expected)

    def test_filter_lost_sphero(self):
        '''
        Simulate where sphero is lost, and now a better spot found
        :return:
        '''
        t = sphero_tracker.SpheroTracker()
        t.running_average = {}

        input_list = \
            [Point(10, 10, 0), # Initial Incorrect Value
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             Point(100, 100, 0),
             ]

        expected = Point(100,100,0)

        for pt in input_list:
            result = t.filter('red', pt)


        self.assertEquals(result, expected)


if __name__ == '__main__':
    unittest.main()
