""" NAO Finder.

A module to find (and track) the NAO using the laser scanner. 
"""

import math, numpy, time

class Robot:
    """ A structure to store a "robot's" data """
    def __init__(self, robotx, roboty, robotr, robotbearing, x, y):
        """ Create a robot """
        self.X = robotx
        self.Y = roboty
        self.R = robotr
        self.Bearing = robotbearing
        self.Orientation = 0
        
        self.ShapeX = x
        self.ShapeY = y
        
        if len(self.ShapeX) < 2:
            self.Width = 0
        else:
            self.Width = numpy.sqrt((self.ShapeX[-1] - self.ShapeX[0])**2 + (self.ShapeY[-1] - self.ShapeY[0])**2)
        
class NAO:
    """ A structure to store the NAO's data """
    
    MAXIMUM_NAO_WIDTH = 32.0         ## maximum width of the NAO in cm
    MINIMUM_NAO_WIDTH = 4.0          ## the smallest width of the NAO in cm
    
    def __init__(self, robot = None):
    
        if robot is None:
            robot = Robot(0, 0, 0, 0, [0], [0])
            
        self.X = robot.X
        self.Y = robot.Y
        self.R = robot.R
        self.Bearing = robot.Bearing
        self.Orientation = 0
        
        self.VX = 0
        self.VY = 0
        self.VOrientation = 0
        self.V = 0
        
        self.ShapeX = robot.ShapeX
        self.ShapeY = robot.ShapeY
        self.Width = robot.Width
        
        self.sigmaX = 3.73
        self.sigmaY = 3.73
        self.sigmaOrientation = 0.09
        self.sigmaVX = 4.45
        self.sigmaVY = 4.45
        self.sigmaVOrientation = 0.6