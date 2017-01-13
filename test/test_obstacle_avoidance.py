import sys
sys.path.insert(0,'../src/')
# Begin From obstacle_avoidance
import rospy
import math
from math import sin, cos

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from collections import namedtuple

Obstacle = namedtuple('Obstacle', ['r', 'theta'])
# End From obstacle_avoidance
from obstacle_avoidance import ObstacleAvoidance
import unittest

class TestCurvatureCalculations(unittest.TestCase):
    def test_left(self):
        # Obstacle = namedtuple('Obstacle', ['r', 'theta'])
        oa = ObstacleAvoidance()
        v = 2
        omega = .1
        originalCurvature = omega/v
        pathWidth = 1
        filteredListOfRThetaPairs = []
        filteredListOfRThetaPairs.append(Obstacle(r=1.6328, theta=-0.4421))
        filteredListOfRThetaPairs.append(Obstacle(r=1.4904, theta=-0.2019))
        filteredListOfRThetaPairs.append(Obstacle(r=1.0792, theta=-0.3143))
        filteredListOfRThetaPairs.append(Obstacle(r=1.4444, theta=-0.3247))
        filteredListOfRThetaPairs.append(Obstacle(r=1.1740, theta=-0.2601))
        filteredListOfRThetaPairs.append(Obstacle(r=1.2565, theta=-0.2686))
        filteredListOfRThetaPairs.append(Obstacle(r=1.5160, theta=-0.5730))
        filteredListOfRThetaPairs.append(Obstacle(r=1.7103, theta=-0.5350))
        filteredListOfRThetaPairs.append(Obstacle(r=1.2089, theta=-0.0008))
        filteredListOfRThetaPairs.append(Obstacle(r=1.7064, theta=-0.5072))
        curvatureToPassObstaclesOnLeft = oa.calculateCurvatureToPassObstaclesOnLeft(originalCurvature, pathWidth, filteredListOfRThetaPairs)
        print(str(curvatureToPassObstaclesOnLeft))
        self.assertTrue(abs(curvatureToPassObstaclesOnLeft-0.8240)<0.001)

    def test_right(self):
        # Obstacle = namedtuple('Obstacle', ['r', 'theta'])
        oa = ObstacleAvoidance()
        v = 2
        omega = .1
        originalCurvature = omega/v
        pathWidth = 1
        filteredListOfRThetaPairs = []
        filteredListOfRThetaPairs.append(Obstacle(r=1.6328, theta=-0.4421))
        filteredListOfRThetaPairs.append(Obstacle(r=1.4904, theta=-0.2019))
        filteredListOfRThetaPairs.append(Obstacle(r=1.0792, theta=-0.3143))
        filteredListOfRThetaPairs.append(Obstacle(r=1.4444, theta=-0.3247))
        filteredListOfRThetaPairs.append(Obstacle(r=1.1740, theta=-0.2601))
        filteredListOfRThetaPairs.append(Obstacle(r=1.2565, theta=-0.2686))
        filteredListOfRThetaPairs.append(Obstacle(r=1.5160, theta=-0.5730))
        filteredListOfRThetaPairs.append(Obstacle(r=1.7103, theta=-0.5350))
        filteredListOfRThetaPairs.append(Obstacle(r=1.2089, theta=-0.0008))
        filteredListOfRThetaPairs.append(Obstacle(r=1.7064, theta=-0.5072))
        curvatureToPassObstaclesOnRight = oa.calculateCurvatureToPassObstaclesOnRight(originalCurvature, pathWidth, filteredListOfRThetaPairs)
        print(str(curvatureToPassObstaclesOnRight))
        self.assertTrue(abs(curvatureToPassObstaclesOnRight-(-1.8228))<0.001)

if __name__ == '__main__':
    unittest.main()
