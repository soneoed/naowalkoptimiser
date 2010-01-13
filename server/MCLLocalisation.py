""" An SIR Particle Filter based localisation system for tracking a robot with ambiguous bearing

Jason Kulk 
"""

from NAO import NAO

import numpy, time

class Localisation:
    X = 0
    Y = 1
    THETA = 2
    XDOT = 3
    YDOT = 4
    THETADOT = 5
    STATE_LENGTH = 6
    
    VEL_PAST_LENGTH = 13
    def __init__(self, M = 1000):
        """ """
        Localisation.NUM_PARTICLES = M
        self.reset = True
        
        self.time = time.time()
        self.previoustime = self.time
        
        self.control = numpy.zeros(3)                                   # the current control
        self.previouscontrol = self.control                             # the previous control
        
        self.measurement = numpy.zeros(Localisation.STATE_LENGTH)       # the current measurement of the state
        self.previousmeasurement = self.measurement                     # the previous measurement of the state
        
        self.previousmeasurementsigma = numpy.zeros(Localisation.STATE_LENGTH)
        
        self.States = numpy.zeros((Localisation.NUM_PARTICLES, Localisation.STATE_LENGTH))      # the (states) particles
        self.PreviousStates = self.States                                                       # the previous state of each particle (used for derivative calculations)
        self.Weights = (1.0/Localisation.NUM_PARTICLES)*numpy.ones(Localisation.NUM_PARTICLES)                                  # the weights of each particle
        self.GUIWeights = (1.0/Localisation.NUM_PARTICLES)*numpy.ones(Localisation.NUM_PARTICLES)                               # the weights of each particle before resampling
        self.State = self.States[0]
        
        # Variables for the control model:
        self.accelerationduration = numpy.array([1.5, 1.5, 0.5])         # the duration an acceleration is applied (s)
        self.accelerationmagnitudes = numpy.array([15, 15, 0.7])/self.accelerationduration       # the magnitude of the accelerations [forward, sideward, turn] (cm/s/s, rad/s)     
        self.accelerations = numpy.zeros((Localisation.NUM_PARTICLES, 3))            # the current acceleration (cm/s/s) for each particle
        self.accelendtimes = numpy.zeros((Localisation.NUM_PARTICLES, 3))            # the times the accelerations will be set to zero given no change in control (s)
        self.startedcontrol = False             # a small hack that will prevent resampling until the control has started
        
        # Variables for additional velocity filtering!
        self.PastVX = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        self.PastVY = list(numpy.zeros(Localisation.VEL_PAST_LENGTH))
        
    def update(self, control, nao):
        """ """
        self.time = time.time()
        self.control = control

        self.measurement = self.__naoToState(nao)
        self.measurementsigma = self.__naoToSigma(nao)
        
        if self.reset:
            self.__initParticles()
            self.reset = False
        else:
            self.predict()

        self.updateWeights()
        self.estimateState()
        self.resample()
        
        self.previoustime = self.time
        self.previousmeasurement = self.measurement
        self.previousmeasurementsigma = self.measurementsigma
        self.PreviousStates = numpy.copy(self.States)
        
    def predict(self):
        """ Updates each of the particles based on system and control model """
        self.modelSystem()
        self.modelControl()
        
    def updateWeights(self):
        """ """
        if not self.startedcontrol:         ## this hack prevents particles from disappearing before the robot starts moving
            return
        
        # calculate variances for the measurement
            # the variance in the velocity is the sum of the current and previous variance in the position measurements
        self.measurementsigma[Localisation.XDOT] = max(4.0, (self.measurementsigma[Localisation.X] + self.previousmeasurementsigma[Localisation.X]))
        self.measurementsigma[Localisation.YDOT] = max(4.0, (self.measurementsigma[Localisation.Y] + self.previousmeasurementsigma[Localisation.Y]))
        self.measurementsigma[Localisation.THETADOT] = max(1.0, 1.5*(self.measurementsigma[Localisation.THETA] + self.previousmeasurementsigma[Localisation.THETA]))
        
        # calculate the weights based on a measurement model
        self.Weights *= self._gauss(self.States[:,Localisation.X] - self.measurement[Localisation.X], self.measurementsigma[Localisation.X])          # 1.73
        self.Weights *= self._gauss(self.States[:,Localisation.Y] - self.measurement[Localisation.Y], self.measurementsigma[Localisation.Y])         # 1.73
        
        # I need a little outlier rejection here:
        anglediff = numpy.fabs(self.measurement[Localisation.THETA] - self.previousmeasurement[Localisation.THETA])
        if anglediff > 5*numpy.pi/12 and anglediff < 7*numpy.pi/12:
            self.measurementsigma[Localisation.THETA] += 1.0
            self.measurementsigma[Localisation.THETADOT] += 15
        elif anglediff > numpy.pi/3 and anglediff < 2*numpy.pi/3:
            self.measurementsigma[Localisation.THETA] += 0.4
            self.measurementsigma[Localisation.THETADOT] += 5
        
            
        self.Weights *= self._gauss(self.States[:,Localisation.THETA] - self.measurement[Localisation.THETA], self.measurementsigma[Localisation.THETA]) + self._gauss(self.States[:,Localisation.THETA] + numpy.pi - self.measurement[Localisation.THETA], self.measurementsigma[Localisation.THETA]) + self._gauss(self.States[:,Localisation.THETA] - numpy.pi - self.measurement[Localisation.THETA], self.measurementsigma[Localisation.THETA]) + self._gauss(self.States[:,Localisation.THETA] + 2*numpy.pi - self.measurement[Localisation.THETA], self.measurementsigma[Localisation.THETA])     # 0.02 + 0.07
        self.Weights *= self._gauss(self.States[:,Localisation.THETADOT] - self.measurement[Localisation.THETADOT], self.measurementsigma[Localisation.THETADOT])
        
        self.Weights *= self._gauss(self.States[:,Localisation.XDOT] - self.measurement[Localisation.XDOT], self.measurementsigma[Localisation.XDOT])   # 2.95 + 1.5
        self.Weights *= self._gauss(self.States[:,Localisation.YDOT] - self.measurement[Localisation.YDOT], self.measurementsigma[Localisation.YDOT])
        controlvector = self.__controlToVelocityVector()
        if controlvector != None:
            diffs = numpy.arctan2(self.States[:,Localisation.YDOT], self.States[:,Localisation.XDOT]) - self.States[:,Localisation.THETA]
            diffs = numpy.arctan2(numpy.sin(diffs), numpy.cos(diffs))           ## I need to normalise the diffs
            self.Weights *= self._gauss(diffs -  self.__controlToVelocityVector(), 0.707)
        
        # normalise the weights so that their sum is one
        sum = numpy.sum(self.Weights)
        if sum != 0:
            self.Weights /= sum
        else:
            print "Oh Noes: All of the weights are zero!"
            print "Measurements:", self.measurement, self.previousmeasurement
            print "State:", self.State
            
            tempweights = (1.0/Localisation.NUM_PARTICLES)*numpy.ones(Localisation.NUM_PARTICLES)
            tempweights *= self._gauss(self.States[:,Localisation.X] - self.measurement[Localisation.X], 3.73)          # 1.73
            print "X:", numpy.average(tempweights)
            tempweights *= self._gauss(self.States[:,Localisation.Y] - self.measurement[Localisation.Y], 3.73)         # 1.73
            print "Y:", numpy.average(tempweights)
            anglediff = numpy.fabs(self.measurement[Localisation.THETA] - self.previousmeasurement[Localisation.THETA])
            if anglediff < 5*numpy.pi/12 or anglediff > 7*numpy.pi/12:
                tempweights *= self._gauss(self.States[:,Localisation.THETA] - self.measurement[Localisation.THETA], 0.09) + self._gauss(self.States[:,Localisation.THETA] + numpy.pi - self.measurement[Localisation.THETA], 0.09) + self._gauss(self.States[:,Localisation.THETA] - numpy.pi - self.measurement[Localisation.THETA], 0.09) + self._gauss(self.States[:,Localisation.THETA] + 2*numpy.pi - self.measurement[Localisation.THETA], 0.09)     # 0.02 + 0.07
                print "THETA:", numpy.average(tempweights)
                self.Weights *= self._gauss(self.States[:,Localisation.THETADOT] - self.measurement[Localisation.THETADOT], 0.6)
                print "THETADOT:", numpy.average(tempweights)
                
            tempweights *= self._gauss(self.States[:,Localisation.XDOT] - self.measurement[Localisation.XDOT], 4.45)   # 2.95 + 1.5
            print "XDOT:", numpy.average(tempweights)
            tempweights *= self._gauss(self.States[:,Localisation.YDOT] - self.measurement[Localisation.YDOT], 4.45)
            print "YDOT:", numpy.average(tempweights)
            
            if controlvector != None:
                diffs = numpy.arctan2(self.States[:,Localisation.YDOT], self.States[:,Localisation.XDOT]) - self.States[:,Localisation.THETA]
                diffs = numpy.arctan2(numpy.sin(diffs), numpy.cos(diffs))           ## I need to normalise the diffs
                tempweights *= self._gauss(diffs -  self.__controlToVelocityVector(), 0.707)
                print "CTRL:", numpy.average(tempweights)
            
            self.__initParticles()
            self.Weights = (1.0/Localisation.NUM_PARTICLES)*numpy.ones(Localisation.NUM_PARTICLES)
        
        self.GUIWeights = numpy.copy(self.Weights)
            
    def __calculateWeight(self, state):
        """ Only use this function for debug purposes. """
        weight = self._gauss(state[Localisation.X] - self.measurement[Localisation.X], 1.73)          # 1.73
        weight *= self._gauss(state[Localisation.Y] - self.measurement[Localisation.Y], 1.73)         # 1.73
        weight *= self._gauss(state[Localisation.THETA] - self.measurement[Localisation.THETA], 0.09) + self._gauss(state[Localisation.THETA] - (self.measurement[Localisation.THETA] - numpy.pi), 0.09) + self._gauss(state[Localisation.THETA] - numpy.pi - self.measurement[Localisation.THETA], 0.09)      # 0.02 + 0.07
        weight *= self._gauss(state[Localisation.XDOT] - self.measurement[Localisation.XDOT], 4.45)   # 2.95 + 1.5
        weight *= self._gauss(state[Localisation.YDOT] - self.measurement[Localisation.YDOT], 4.45)
        weight *= numpy.where(numpy.fabs(state[Localisation.THETADOT] - self.measurement[Localisation.THETADOT]) < 2, self._gauss(state[Localisation.THETADOT] - self.measurement[Localisation.THETADOT], 0.7), 1)
        controlvector = self.__controlToVelocityVector()
        if controlvector != None:
            diff = numpy.arctan2(state[Localisation.YDOT], state[Localisation.XDOT]) - state[Localisation.THETA]
            diff = numpy.arctan2(numpy.sin(diff), numpy.cos(diff))           ## I need to normalise the diffs
            weight *= self._gauss(diff -  self.__controlToVelocityVector(), 0.707)
        return weight

        
    def resample(self):
        """ """
        # An SIS filter resamples only when necessary
        Neff = 1.0/numpy.sum(self.Weights**2)
        Ns = Localisation.NUM_PARTICLES
        if self.startedcontrol:# and Neff < 0.3*Ns:
            #print "Resample:", Neff,  "<", Ns 
            NsInv = 1.0/Ns
            c = numpy.cumsum(self.Weights)
            u = NsInv*numpy.arange(Ns) + numpy.random.uniform(0, NsInv)
            
            # I want to put in a fancy velocity check in the resample that spins particles around that are moving in the opposite direction
            controlvector = self.__controlToVelocityVector()
            if controlvector != None:
                diffs = numpy.arctan2(self.States[:,Localisation.YDOT], self.States[:,Localisation.XDOT]) - self.States[:,Localisation.THETA]
                diffs = numpy.arctan2(numpy.sin(diffs), numpy.cos(diffs))           ## I need to normalise the diffs
                diffs = numpy.fabs(diffs - self.__controlToVelocityVector())
                vc = numpy.fabs(diffs - numpy.pi) < 0.15
            
            i = 0
            #print "Pre resample:"
            #print self.States[:,0:3]
            for j in range(Ns):
                while u[j] > c[i]:
                    i = i + 1
                self.States[j] = numpy.copy(self.States[i])
                self.PreviousStates[j] = numpy.copy(self.PreviousStates[i])
                self.accelerations[j] = numpy.copy(self.accelerations[i])
                self.accelendtimes[j] = numpy.copy(self.accelendtimes[i])
                if controlvector != None and vc[i]:
                    print "Flipping Particle:", j, self.States[j, Localisation.THETA]
                    #print self.__calculateWeight(self.States[j]), self.__calculateWeight(self.States[i])
                    self.States[j, Localisation.THETA] = numpy.arctan2(numpy.sin(self.States[j,Localisation.THETA] - numpy.pi), numpy.cos(self.States[j,Localisation.THETA] - numpy.pi))
                    self.PreviousStates[j, Localisation.THETA] = numpy.arctan2(numpy.sin(self.PreviousStates[j,Localisation.THETA] - numpy.pi), numpy.cos(self.PreviousStates[j,Localisation.THETA] - numpy.pi))
            
            #print "Post resample:"
            #print self.States[:,0:3]
            self.Weights = NsInv*numpy.ones(Ns)
        
    def modelSystem(self):
        """ Updates each particle based on the system model """
        dt = self.time - self.previoustime
        if not self.startedcontrol:
            sdxdot = 0.05                    
            sdydot = 0.05
            sdthetadot = 0.01
        else:
            sdxdot = 1.5                    # these weights can probably be a bit lower!
            sdydot = 1.5
            sdthetadot = 0.2
        
        xdot = self.PreviousStates[:,Localisation.XDOT] + numpy.random.normal(0, sdxdot, size=self.PreviousStates.shape[0])
        ydot = self.PreviousStates[:,Localisation.YDOT] + numpy.random.normal(0, sdydot, size=self.PreviousStates.shape[0])
        thetadot = self.PreviousStates[:,Localisation.THETADOT] + numpy.random.normal(0, sdthetadot/2.0, size=self.PreviousStates.shape[0])
        
        self.States[:,Localisation.XDOT] = xdot*numpy.cos(thetadot*dt) - ydot*numpy.sin(thetadot*dt)
        self.States[:,Localisation.YDOT] = ydot*numpy.cos(thetadot*dt) + xdot*numpy.sin(thetadot*dt)
        self.States[:,Localisation.THETADOT] = thetadot + numpy.random.normal(0, sdthetadot/2.0, size=self.PreviousStates.shape[0])
        
        self.States[:,Localisation.THETA] = self.PreviousStates[:,Localisation.THETA] + thetadot*dt
        self.States[:,Localisation.X] = self.PreviousStates[:,Localisation.X] + xdot*dt*numpy.cos(self.States[:,Localisation.THETADOT]*dt) - ydot*dt*numpy.sin(self.States[:,Localisation.THETADOT]*dt)
        self.States[:,Localisation.Y] = self.PreviousStates[:,Localisation.Y] + ydot*dt*numpy.cos(self.States[:,Localisation.THETADOT]*dt) + xdot*dt*numpy.sin(self.States[:,Localisation.THETADOT]*dt)
        
        # make sure that theta is between +/- pi
        self.States[:,Localisation.THETA] = numpy.arctan2(numpy.sin(self.States[:,Localisation.THETA]), numpy.cos(self.States[:,Localisation.THETA]))
        
    def modelControl(self):
        """ Updates each particle based on the control model """
        # my model for control, is that a change in control will effect the state by
        # introducing a constant acceleration over the next 1 second (2 steps)
        deltacontrol = self.control - self.previouscontrol
        
        sdx = 1             # noise on estimate of acceleration magnitude (in cm/s/s)
        sdy = 1             # noise on estimate of acceleration magnitude (in cm/s/s)
        sdtheta = 0.2       # noise on estimate of acceleration magnitude (in rad/s/s)
        
        if self.control[0] == 0 and self.previouscontrol[0] != 0:       # if I was previously walking and now I want to stop, deaccelerate
            self.accelerations[:,0] = (-self.PreviousStates[:,Localisation.XDOT]/self.accelerationduration[0]) + numpy.random.normal(0, sdx, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,0] = self.time + self.accelerationduration[0]
            self.accelerations[:,1] = (-self.PreviousStates[:,Localisation.YDOT]/self.accelerationduration[0]) + numpy.random.normal(0, sdy, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,1] = self.time + self.accelerationduration[0]
            self.accelerations[:,2] = (-self.PreviousStates[:,Localisation.THETADOT]/self.accelerationduration[2]) + numpy.random.normal(0, sdtheta, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,2] = self.time + self.accelerationduration[0]
        
        elif self.control[0] !=0 and self.previouscontrol[0] == 0:      # if I was previously stopped and now I want to start, accelerate
            self.startedcontrol = True
            self.accelerations[:,0] = self.accelerationmagnitudes[0]*numpy.cos(self.PreviousStates[:,Localisation.THETA]) + numpy.random.normal(0, sdx, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,0] = self.time + self.accelerationduration[0]
            self.accelerations[:,1] = self.accelerationmagnitudes[0]*numpy.sin(self.PreviousStates[:,Localisation.THETA]) + numpy.random.normal(0, sdy, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,1] = self.time + self.accelerationduration[0]
        
        # put a bit of spin on the robot if the desired bearing changes
        if abs(deltacontrol[1]) > 0 and abs(self.control[1]) > 0.1:
            self.accelerations[:,2] += deltacontrol[1] + numpy.random.normal(0, sdtheta, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,2] = self.time + self.accelerationduration[2]
        
        # put a bit of spin on the robot if the final orientation changes
        if self.control[2] < 1000 and abs(self.control[0]) < 10 and abs(deltacontrol[2]) > 0:
            if self.previouscontrol[2] > 1000:
                self.accelerations[:,2] += self.control[2] + numpy.random.normal(0, sdtheta, size=self.PreviousStates.shape[0])
            else:
                self.accelerations[:,2] += deltacontrol[2] + numpy.random.normal(0, sdtheta, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,2] = self.time + self.accelerationduration[2]
        
        self.accelerations = numpy.where(self.accelendtimes > self.time, self.accelerations, 0)
        numpy.clip(self.accelerations[:,2], -0.7, 0.7, self.accelerations[:,2])
        #numpy.clip(self.accelerations[:,1], -20, 20, self.accelerations[:,1])
        #numpy.clip(self.accelerations[:,0], -20, 20, self.accelerations[:,0])
        # calculate the controls contribution to the state velocity
        self.States[:,Localisation.XDOT:] += self.accelerations*(self.time - self.previoustime)
        
        self.previouscontrol = self.control
        
    def estimateState(self):
        """ Updates the estimate of the state """
        best = numpy.argmax(self.Weights)
        beststate = self.States[best,:]
        
        #print "Best State:", beststate
        
        cond = (numpy.sum(numpy.fabs(self.States - beststate), axis=1) < 1)
        beststates = numpy.compress(cond, self.States, axis=0)
        bestweights = numpy.compress(cond, self.Weights)
        
        #print "States", self.States
        #print "States within window:", cond
        #print "States close to best", len(beststates), beststates
        #print "Weights close to best", bestweights
        
        #print "Product:", (bestweights*beststates.T).T
        bestweights /= numpy.sum(bestweights)
        self.State = numpy.sum((bestweights*beststates.T).T, axis=0)
        #print "Estimate:", self.State
        
        #print numpy.fabs(numpy.arctan2(self.State[Localisation.YDOT], self.State[Localisation.XDOT]) - self.State[Localisation.THETA]) -  self.__controlToVelocityVector()
        
        if numpy.isnan(self.State[0]):
            print "FAIL"
        self.__updateAttributesFromState()
        
    def __initParticles(self):
        """ Initialises self.Particles to contain Localisation.NUM_PARTICLES particles around the current measurement """
        print "Initialising Particles around", self.measurement
        self.States = numpy.zeros((Localisation.NUM_PARTICLES, Localisation.STATE_LENGTH))
        self.States += self.measurement
        # I know for certain that at the beginning the robot is not moving, so all of the velocities should be zero. The Position however should get some noise
        self.States[:,Localisation.X] += numpy.random.normal(0, 1.73, size=self.States.shape[0])
        self.States[:,Localisation.Y] += numpy.random.normal(0, 1.73, size=self.States.shape[0])
        self.States[:,Localisation.THETA] += numpy.random.normal(0, 0.09, size=self.States.shape[0])
        # now swap half of the orientations
        self.States[:, Localisation.THETA] = numpy.where(numpy.random.uniform(0,1, size=self.States.shape[0]) < 0.5, self.States[:, Localisation.THETA], self.States[:, Localisation.THETA] - numpy.pi)
        #print self.States
        
    def __getStateNearMeasurement(self):
        """ """
        state = self.measurement + numpy.random.normal(0, 0.15, len(self.measurement))
        if numpy.random.uniform(0,1) < 0.5:
            state[Localisation.THETA] -= numpy.pi
        return state
        
    def __naoToState(self, nao):
        state = numpy.zeros(Localisation.STATE_LENGTH)
        if nao != None:
            state[Localisation.X] = nao.X
            state[Localisation.Y] = nao.Y
            state[Localisation.THETA] = nao.Orientation
            state[Localisation.XDOT] = nao.VX
            state[Localisation.YDOT] = nao.VY
            state[Localisation.THETADOT] = nao.VOrientation
        else:
            state[Localisation.X] = self.previousmeasurement[Localisation.X]
            state[Localisation.Y] = self.previousmeasurement[Localisation.Y]
            state[Localisation.THETA] = self.previousmeasurement[Localisation.THETA]
            
        return state
        
    def __naoToSigma(self, nao):
        """ """
        state = numpy.zeros(Localisation.STATE_LENGTH)
        if nao != None:
            state[Localisation.X] = nao.sigmaX
            state[Localisation.Y] = nao.sigmaY
            state[Localisation.THETA] = nao.sigmaOrientation
            state[Localisation.XDOT] = nao.sigmaVX
            state[Localisation.YDOT] = nao.sigmaVY
            state[Localisation.THETADOT] = nao.sigmaVOrientation
        else:
            state[Localisation.X] = 3.73
            state[Localisation.Y] = 3.73
            state[Localisation.THETA] = 0.09
            state[Localisation.XDOT] = 4.45
            state[Localisation.YDOT] = 4.45
            state[Localisation.THETADOT] = 0.6
            
        return state
        
    def __updateAttributesFromState(self):
        """ I have a bunch of convienent attributes for accessing the state. I need to keep them for backward compatiblity purposes. """
        self.X = self.State[Localisation.X]
        self.Y = self.State[Localisation.Y]
        self.Orientation = self.State[Localisation.THETA]
        # I want the velocities to be robot relative
        vx = self.State[Localisation.XDOT]
        vy = self.State[Localisation.YDOT]
        vx = self.measurement[Localisation.XDOT]        # I am usually happier using the actual velocity measurements
        vy = self.measurement[Localisation.YDOT]
        relativevx = vx*numpy.cos(self.Orientation) + vy*numpy.sin(self.Orientation)
        relativevy = -vx*numpy.sin(self.Orientation) + vy*numpy.cos(self.Orientation)
        self.VX = self._filter(self.PastVX, relativevx)
        self.VY = self._filter(self.PastVY, relativevy)
        self.VOrientation = self.State[Localisation.THETADOT]
        self.V = numpy.sqrt(self.VX**2 + self.VY**2)
        
    def __controlToVelocityVector(self):
        """ Returns the velocity vector expected given the control.
            Returns None when I am not sure what to expect """
            
        if self.control[0] < 0:
            absdirection = abs(self.control[1])
            if absdirection < 0.19:
                return 0
            elif absdirection < 0.5:
                return self.control[1]
            else:
                return None
        else:
            return None
        
    def _gauss(self, x, sigma):
        return (1.0/numpy.sqrt(2*numpy.pi*sigma))*numpy.exp(-(x**2)/(2*sigma**2))
        
    def _filter(self, pastvalues, measurement):
        """ """
        pastvalues.append(measurement)
        pastvalues.pop(0)
        return numpy.average(pastvalues, weights=numpy.arange(len(pastvalues)))
        

if __name__ == '__main__':
    import matplotlib
    matplotlib.use('WXAgg')
    matplotlib.rcParams['toolbar'] = 'None'
    
    import pylab, psyco, wx
    psyco.full()
    
    x = list()
    y = list()
    o = list()
    localisation = Localisation(1000)
    #pylab.figure()
    #p = numpy.arange(-numpy.pi, numpy.pi, 0.01)
    #pylab.plot(p, localisation._gauss(p - 1, 0.02) + localisation._gauss(p + 2.14, 0.02))
    #pylab.figure()
    loopcount = 0
    control = numpy.zeros(3)
    
    ax = pylab.subplot(111)
    canvas = ax.figure.canvas

    particleplot, = pylab.plot([0,0],[0,0], marker='o', color='k', linewidth=0, markersize=2, animated=True)
    estimateplot, = pylab.plot([0,0],[0,0], marker='o', animated=True)
    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    canvas.draw()
    canvas.gui_repaint()
    
    def update_plot(*args):
        """ hmm """
        global control, loopcount, localisation
        if update_plot.background is None:
           update_plot.background = canvas.copy_from_bbox(ax.bbox)
           
        starttime = time.time()
        localisation.update(control, None)
        x.append(localisation.State[0])
        y.append(localisation.State[1])
        o.append(localisation.State[2])
        loopcount += 1
        if loopcount == 2:
            print "Starting"
            control = numpy.array([-1,0.3,0])
        elif loopcount == 50:
            control = numpy.array([0,0,0])            
        canvas.restore_region(update_plot.background)
        estimateplot.set_data(x,y)
        particleplot.set_data(numpy.array(localisation.States[:,Localisation.Y]), numpy.array(localisation.States[:,Localisation.X]))
        ax.draw_artist(particleplot)
        #ax.draw_artist(estimateplot)
        canvas.blit(ax.bbox)
        time.sleep(max(0,0.1 - (time.time() - starttime)))
        wx.WakeUpIdle()

    update_plot.background = None
    wx.EVT_IDLE(wx.GetApp(), update_plot)
    pylab.show()
