""" """
import numpy, time

from NAO import NAO

class Localisation:
    POS_PAST_LENGTH = 3
    VEL_PAST_LENGTH = 8
    WALK_PAST_LENGTH = 20
    def __init__(self):
        """ """
        self.PastX = list(numpy.zeros(Localisation.POS_PAST_LENGTH))
        self.PastY = list(numpy.zeros(Localisation.POS_PAST_LENGTH))
        self.PastOrientation = list(numpy.zeros(Localisation.POS_PAST_LENGTH))
        self.PreviousX = 0
        self.PreviousY = 0
        self.PreviousOrientation = 0
        
        self.PastVX = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        self.PastVY = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        self.PastVOrientation = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        self.PastV = list(numpy.zeros(Localisation.WALK_PAST_LENGTH))
        
        self.Time = time.time()
        self.PreviousTime = self.Time
        
        self.Reset = True
        
        # convenience aliases (effectively state get methods)
        self.X = 0
        self.Y = 0
        self.R = 0
        self.Bearing = 0
        self.Orientation = 0
        self.VX = 0
        self.VY = 0
        self.V = 0
        self.VOrientation = 0
        
    def update(self, control, measurement):
        """ """
        self.Control = control
        self.Measurement = measurement
        self.Time = time.time()
        
        if self.Reset == True:
            self.reset(self.Measurement)
            self.Reset = False
    
        self.X = self.__filter(self.PastX, self.Measurement.X)
        self.Y = self.__filter(self.PastY, self.Measurement.Y)
        self.R = numpy.sqrt(self.X**2 + self.Y**2)
        self.Bearing = numpy.arctan2(self.Y, self.X)
        self.Orientation = self.__filterOrientation(self.Measurement.Orientation)
        
        self.VX = self.__filter(self.PastVX, (self.X - self.PreviousX)/(self.Time - self.PreviousTime))
        self.VY = self.__filter(self.PastVY, (self.Y - self.PreviousY)/(self.Time - self.PreviousTime))
        self.VOrientation = self.__filter(self.PastVOrientation, (self.Orientation - self.PreviousOrientation)/(self.Time - self.PreviousTime))
        self.V = self.__filter(self.PastV, numpy.sqrt(self.VX**2 + self.VY**2))
            
        self.PreviousX = self.X
        self.PreviousY = self.Y
        self.PreviousOrientation = self.Orientation
        self.PreviousTime = self.Time
    
    def reset(self, measurement):
        """ """
        self.PastX = list(numpy.repeat(measurement.X, Localisation.POS_PAST_LENGTH))
        self.PastY = list(numpy.repeat(measurement.Y, Localisation.POS_PAST_LENGTH))
        self.PastOrientation = list(numpy.repeat(measurement.Orientation, Localisation.POS_PAST_LENGTH))
        
        self.PastVX = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        self.PastVY = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        self.PastVOrientation = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        self.PastV = list(numpy.zeros(Localisation.WALK_PAST_LENGTH))
        
        self.PreviousX = measurement.X
        self.PreviousY = measurement.Y
        self.PreviousOrientation = measurement.Orientation
        
    def __filter(self, pastvalues, measurement):
        """ """
        pastvalues.append(measurement)
        pastvalues.pop(0)
        return numpy.average(pastvalues)
        
    def __filterOrientation(self, measurement):
        """ """
        # firstly decide which measurement is closest to the current orientation
        a = self.Measurement.Orientation
        b = self.Measurement.Orientation - numpy.pi
        if numpy.fabs(self.Orientation - a) < numpy.fabs(self.Orientation - b):
            measurement = a
        else:
            measurement = b
        
        self.PastOrientation.append(measurement)
        self.PastOrientation.pop(0)
        # now I need to be extremely careful to average the past values correctly
        return self.Measurement.Orientation#numpy.average(self.PastOrientation)
        
        