""" NAO Finder.

A module to find (and track) the NAO using the laser scanner. 
"""
from NAO import Robot, NAO

import math, numpy, time

class NAOFinder:
    """ A module to find (and track) the NAO using a laser scanner """
    def __init__(self, localisation = None):
        """ Create a NAO finder.
        """
        self.MAXIMUM_NAO_WIDTH = NAO.MAXIMUM_NAO_WIDTH         ## maximum width of the NAO in cm
        self.MINIMUM_NAO_WIDTH = NAO.MINIMUM_NAO_WIDTH          ## the smallest width of the NAO in cm
        
        self.Localisation = localisation
        
        self._clearNAOLocation()
        
        self.LeftEdges = list()
        self.RightEdges = list()
        self.Robots = list()
        
        self.PreviousTime = time.time()
        self.Time = time.time()
        
    def _clearNAOLocation(self):
        """ """
        self.NaoX = 0
        self.NaoY = 0
        self.NaoOrientation = 0
        self.NaoVelocityX = 0
        self.NaoVelocityY = 0
        self.NaoVelocityOrientation = 0
        self.NaoVelocity = 0
        
        self.NaoPreviousX = 0
        self.NaoPreviousY = 0
        self.NaoPreviousOrientation = 0
        
        self.ShapeX = numpy.array([0, 0])
        self.ShapeY = numpy.array([0, 0])
        self.slope = 0
        self.intercept = 0
        
        self.NAO = NAO()
        self.PreviousNAO = self.NAO
        
    def findNAO(self, polardata, cartesiandata):
        """ Finds the position and velocity of the NAO
        @param polardata has the following format: rangedata, angledata
        @param cartesiandata has the following format: xdata, ydata
        """
        self.PolarData = numpy.array(polardata)
        self.CartesianData = numpy.array(cartesiandata)
        self._findEdges()
        self._findRobots()
        self._findNAO()
        return self.NAO
        
    def _findEdges(self):
        """ Finds edges in the rangedata
        """
        EDGE_SIZE = 8.0          ## the minimum distance between two consecutive points that constitutes an edge (cm)
        rangedata = self.PolarData[0]
        
        leftedges = list()
        self.LeftEdges = list()
        rightedges = list()
        self.RightEdges = list()
        
        ## rangedata[0] is the rightmost data point, and rangedata[len(rangedata)] is the leftmost data point
        for i in range(1, len(rangedata)):
            if (rangedata[i] - rangedata[i-1] > EDGE_SIZE):        ## a near->far edge (left edge)
                leftedges.append(i-1)
            elif (rangedata[i] - rangedata[i-1] < -EDGE_SIZE):   ## a far->near edge (right edge)
                rightedges.append(i)
        
        ## I want to merge all right edges that are less than the maximum width of the NAO together.
        ## I keep the first edge!
        if len(rightedges) != 0:
            self.RightEdges.append(rightedges[0])       ## I have to keep the first edge
        for rightedge in rightedges[1:]:
            distance = self.__calculateDistanceBetweenEdges(rightedge, self.RightEdges[-1])
            pointssep = rightedge - self.RightEdges[-1]
            if distance > self.MAXIMUM_NAO_WIDTH:
                self.RightEdges.append(rightedge)
            else:
                pass
                #print "Merging right edge", rightedge , "with", self.RightEdges[-1]
        
        ## Similarly, I want the merge the left edges
        ## This time I want to keep the last edge
        leftedges.reverse()                       ## the logic is easier in reverse, just remember to reverse again at the end
        if len(leftedges) != 0:
            self.LeftEdges.append(leftedges[0])       ## I have to keep the last edge
        for leftedge in leftedges[1:]:
            distance = self.__calculateDistanceBetweenEdges(leftedge, self.LeftEdges[-1])
            pointssep = self.LeftEdges[-1] - leftedge
            if distance > self.MAXIMUM_NAO_WIDTH:
                self.LeftEdges.append(leftedge)
            else:
                pass
                #print "Merging left edge", leftedge , "with", self.LeftEdges[-1]
        self.LeftEdges.reverse()        
        
        #print self.RightEdges, self.LeftEdges
                
    def _findRobots(self):
        """ Finds the robots amoung the edges found
        """
        ## for each right edge find the next closest left edge. This forms an edge pair that could be robot 
        self.Robots = list()
        if len(self.RightEdges) == 0 or len(self.LeftEdges) == 0:
            return
            
        for rightedge in self.RightEdges:
            leftedge = self.LeftEdges[0]
            i = 1
            while leftedge < rightedge:
                if i >= len(self.LeftEdges):
                    break
                leftedge = self.LeftEdges[i]
                i = i + 1
                
            ## now calculate the distance between the two edges
            distance = self.__calculateDistanceBetweenEdges(leftedge, rightedge)
            
            if distance > self.MINIMUM_NAO_WIDTH and distance < self.MAXIMUM_NAO_WIDTH:
                x = self.CartesianData[0,rightedge:leftedge+1]
                y = self.CartesianData[1,rightedge:leftedge+1]
                r = self.PolarData[0,rightedge:leftedge+1]
                c = numpy.less(r, 409.5)
                x = numpy.compress(c, x)
                y = numpy.compress(c, y)                
                robotx = self.__averageObjectDistance(x)
                roboty = self.__averageObjectDistance(y)
                c = numpy.logical_and(numpy.less(numpy.fabs(x - robotx), self.MAXIMUM_NAO_WIDTH), numpy.less(numpy.fabs(y - roboty), self.MAXIMUM_NAO_WIDTH))
                x = numpy.compress(c, x)
                y = numpy.compress(c, y)
                robotr = math.sqrt(robotx**2 + roboty**2)
                robotbearing = math.atan2(roboty, robotx)
                self.Robots.append(Robot(robotx, roboty, robotr, robotbearing, x, y))
        
    def _findNAO(self):
        """
        """
        self.Time = time.time()
        
        if self.NaoX == 0 and self.NaoY == 0:       ## then we are presently tracking no NAO's
            self.__findNAOOnInit()
        else:                                           ## if we are already tracking a NAO pick the robot closest to the NAO
            mindistance = 70
            thenao = None
            for robot in self.Robots:
                distance = math.sqrt((robot.X - self.NaoX)**2 + (robot.Y - self.NaoY)**2)     ## the distance from the current position of the NAO
                if distance < mindistance:
                    mindistance = distance
                    thenao = robot
            
            if thenao != None:
                velocityx = (thenao.X - self.NaoX)/(self.Time - self.PreviousTime)
                velocityy = (thenao.Y - self.NaoY)/(self.Time - self.PreviousTime)
                velocity = math.sqrt(velocityx**2 + velocityy**2)
                if velocity < 100.0:
                    self.__updateNAO(thenao)
                else:
                    self.__updateFallenNAO()
            else:
                self.__updateFallenNAO()
                    
                    
    def __findNAOOnInit(self):
        """ Assumes the NAO is positioned approximately in front of the laser scanner """
        mincost = 1000
        thenao = None
        for robot in self.Robots:
            cost = numpy.fabs(robot.Bearing) + robot.R/100.0
            if cost < mincost:
                mincost = cost
                thenao = robot
        
        if thenao != None:
            self.NaoX = thenao.X
            self.NaoY = thenao.Y
            self.NaoPreviousX = self.NaoX
            self.NaoPreviousY = self.NaoY
            self.NaoOrientation, sigmaOrientation = self.__determineOrientation(thenao)
            self.NaoPreviousO = self.NaoOrientation
            self.PreviousTime = 0
            
            self.NAO = NAO(thenao)
            self.NAO.Orientation = self.NaoOrientation
            self.NAO.VX = self.NaoVelocityX
            self.NAO.VY = self.NaoVelocityY
            self.NAO.VOrientation = self.NaoVelocityOrientation
            self.NAO.V = self.NaoVelocity
        else:
            self._clearNAOLocation()
            
    def __updateNAO(self, newnao):
        """ """
        self.NaoPreviousX = self.NaoX
        self.NaoPreviousY = self.NaoY
        self.NaoPreviousOrientation = self.NaoOrientation
        self.NaoX = newnao.X
        self.NaoY = newnao.Y
        self.NaoVelocityX = (self.NaoX - self.NaoPreviousX)/(self.Time - self.PreviousTime)
        self.NaoVelocityY = (self.NaoY - self.NaoPreviousY)/(self.Time - self.PreviousTime)
        self.NaoVelocity = math.sqrt(self.NaoVelocityX**2 + self.NaoVelocityY**2)
        self.NaoOrientation, sigmaOrientation = self.__determineOrientation(newnao)

        # I need to be careful with the orientation velocity because it can wrap around 0->PI or PI->0, this produces a huge velocity and kills localisation
        # case 1: previous is 0.1 this is 3.1 => diff = 3
        if (self.NaoOrientation - self.NaoPreviousOrientation) > numpy.pi/2:
            deltaOrientation = self.NaoOrientation - numpy.pi - self.NaoPreviousOrientation
        # case 2: previous is 3.1 this is 0.1 => diff = -3
        elif (self.NaoOrientation - self.NaoPreviousOrientation) < -numpy.pi/2:
            deltaOrientation = self.NaoOrientation - (self.NaoPreviousOrientation - numpy.pi)
        else:
            deltaOrientation = self.NaoOrientation - self.NaoPreviousOrientation
        self.NaoVelocityOrientation = deltaOrientation/(self.Time - self.PreviousTime)
        self.PreviousTime = self.Time
        
        self.NAO = NAO(newnao)
        self.NAO.Orientation = self.NaoOrientation
        self.NAO.VX = self.NaoVelocityX
        self.NAO.VY = self.NaoVelocityY
        self.NAO.VOrientation = self.NaoVelocityOrientation
        self.NAO.V = self.NaoVelocity
        
        R = numpy.sqrt(self.NAO.X**2 + self.NAO.Y**2)
        self.NAO.sigmaX = 0.015*R
        self.NAO.sigmaY = 0.015*R 
        self.NAO.sigmaOrientation = sigmaOrientation
        
    def __updateFallenNAO(self):
        """ """
        self.NAO.VX = 0
        self.NAO.VY = 0
        self.NAO.V = 0
                
    def __calculateDistanceBetweenEdges(self, firstedge, secondedge):
        """ Returns the distance in mm between the two edges.
        @param firstedge is the index of the first edge
        @param secondedge is the index of the second edge
        """
        x = self.CartesianData[0][firstedge] - self.CartesianData[0][secondedge]
        y = self.CartesianData[1][firstedge] - self.CartesianData[1][secondedge]
        return math.sqrt(x**2 + y**2)
    
    def __averageObjectDistance(self, values):
        """ Returns the average of the values, being careful not to include outliers (such as the gap between the torso and the arms) """
        if len(values) == 0:
            return 409.6
        else:
            avg = numpy.average(values)
            sum = 0
            entries = 0
            for value in values:
                if math.fabs(value - avg) < self.MAXIMUM_NAO_WIDTH:         ## remove outliers from the calculation
                    entries = entries + 1
                    sum = sum + value
            if entries == 0:
                return 409.6
            else:
                return sum/entries
            
    def __determineOrientation(self, robot):
        """ """
        self.ShapeX = robot.ShapeX
        self.ShapeY = robot.ShapeY
        
        x = self.ShapeX
        y = self.ShapeY
        tx = x*numpy.cos(robot.Bearing) + y*numpy.sin(robot.Bearing)
        ty = -x*numpy.sin(robot.Bearing) + y*numpy.cos(robot.Bearing)
        
        if robot.Width > 15:
            p, residuals, rank, singular, rcond = numpy.polyfit(ty, tx, deg=1, full=True)
            self.slope, self.intercept = p
            sigma = max(0.1, 0.015*residuals/len(tx))
            a = numpy.arctan(1/self.slope)
            self.slope = (self.slope - numpy.tan(robot.Bearing))/(1 + self.slope*numpy.tan(robot.Bearing))
            self.intercept = self.intercept*numpy.sin(a)/numpy.sin(a + robot.Bearing)
            ret = numpy.pi/2 + numpy.arctan(1.0/self.slope)# + robot.Bearing
        else:
            ret = numpy.pi/2 + robot.Bearing
            sigma = 0.3
            #print "Narrow Robot:", robot.Width, "Orientation:", ret
        
        if ret > numpy.pi:
            ret = ret - numpy.pi
        elif ret < 0: 
            ret = ret + numpy.pi
        return ret, sigma
        
        
    def __normaliseAngle(self, angle):
        """ """
        return numpy.arctan2(numpy.sin(angle), numpy.cos(angle))
        
            
        