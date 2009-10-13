""" An SIR Particle Filter based localisation system for tracking a robot with ambiguous bearing

Jason Kulk 
"""

from NAO import NAO

import numpy, time

class Localisation:
    THETA = 0
    THETADOT = 1
    STATE_LENGTH = 2
    NUM_PARTICLES = 1000
    def __init__(self):
        """ """
        self.reset = True
        
        self.time = time.time()
        self.previoustime = self.time
        
        self.control = numpy.zeros(3)                                   # the current control
        self.previouscontrol = self.control                             # the previous control
        
        self.measurement = numpy.zeros(Localisation.STATE_LENGTH)       # the current measurement of the state
        self.previousmeasurement = self.measurement                     # the previous measurement of the state
        
        self.States = numpy.zeros((Localisation.NUM_PARTICLES, Localisation.STATE_LENGTH))      # the (states) particles
        self.PreviousStates = self.States                                                       # the previous state of each particle (used for derivative calculations)
        self.Weights = numpy.zeros(Localisation.NUM_PARTICLES)                                  # the weights of each particle
        self.State = self.States[0]                                                             # the estimate of the state
        
        # Variables for the control model:
        self.accelerationduration = [2.0, 2.0, 1.0]         # the duration an acceleration is applied (s)
        self.accelerationmagnitudes = [7.5, 5.0, 0.7]       # the magnitude of the accelerations [forward, sideward, turn] (cm/s/s, rad/s)            
        self.accelerations = numpy.zeros((Localisation.NUM_PARTICLES, 3))            # the current acceleration (cm/s/s) for each particle
        self.accelendtimes = numpy.zeros((Localisation.NUM_PARTICLES, 3))            # the times the accelerations will be set to zero given no change in control (s)
        
    def update(self, control, nao):
        """ """
        self.time = time.time()
        self.control = control
        self.measurement = self.__naoToState(nao)
        
        if self.reset:
            self.__initParticles()
            self.reset = False
        else:
            self.predict()

        self.updateWeights()
        self.resample()
        
        self.estimateState()    
        
        self.previoustime = self.time
        self.PreviousStates = self.States
        
    def predict(self):
        """ Updates each of the particles based on system and control model """
        self.modelSystem()
        self.modelControl()
        
    def updateWeights(self):
        """ """
        # calculate the weights based on a measurement model
        self.Weights = self.__gauss(self.States[:,Localisation.THETA] - self.measurement[Localisation.THETA], 0.02) + self.__gauss(self.States[:,Localisation.THETA] - (self.measurement[Localisation.THETA] - numpy.pi), 0.02)
        self.Weights *= self.__gauss(self.States[:,Localisation.THETADOT] - self.measurement[Localisation.THETADOT], 0.25)
        
        # normalise the weights so that their sum is one
        sum = numpy.sum(self.Weights)
        if sum != 0:
            self.Weights /= sum
        
    def resample(self):
        """ """
        # An SIS filter resamples only when necessary
        Neff = 1.0/numpy.sum(self.Weights**2)
        Ns = Localisation.NUM_PARTICLES
        if Neff < 0.1*Ns:
            NsInv = 1.0/Ns
            c = numpy.cumsum(self.Weights)
            u = NsInv*numpy.arange(Ns) + numpy.random.uniform(0, NsInv)
            i = 0
            #print "Pre resample:"
            #print self.States[:,0:3]
            for j in range(Ns):
                while u[j] > c[i]:
                    i = i + 1
                self.States[j] = self.States[i]
                self.PreviousStates[j] = self.PreviousStates[i]
                self.accelerations[j] = self.accelerations[i]
                self.accelendtimes[j] = self.accelendtimes[i]
            
            #print "Post resample:"
            #print self.States[:,0:3]
            self.Weights = NsInv*numpy.ones(Ns)
        
    def modelSystem(self):
        """ Updates each particle based on the system model """
        dt = self.time - self.previoustime
        sdthetadot = 0.25
        
        self.States[:,Localisation.THETADOT] = self.PreviousStates[:,Localisation.THETADOT] + numpy.random.normal(0, sdthetadot, size=self.PreviousStates.shape[0])
        self.States[:,Localisation.THETA] = self.PreviousStates[:,Localisation.THETA] + self.States[:,Localisation.THETADOT]*dt
        
        self.States[:,Localisation.THETA] = numpy.arctan2(numpy.sin(self.States[:,Localisation.THETA]), numpy.cos(self.States[:,Localisation.THETA]))
        
    def modelControl(self):
        """ Updates each particle based on the control model """
        # my model for control, is that a change in control will effect the state by
        # introducing a constant acceleration over the next 1 second (2 steps)
        deltacontrol = self.control - self.previouscontrol
        
        sdtheta = 0.2       # noise on estimate of acceleration magnitude (in rad/s/s)
        
        # put a bit of spin on the robot if the desired bearing changes
        if abs(deltacontrol[1]) > 0:
            self.accelerations[:,2] += (1/self.accelerationduration[2])*deltacontrol[1] + numpy.random.normal(0, sdtheta, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,2] = self.time + self.accelerationduration[2]
        
        # put a bit of spin on the robot if the final orientation changes
        if self.control[2] < 1000 and abs(self.control[0]) < 10 and abs(deltacontrol[2]) > 0:
            if self.previouscontrol[2] > 1000:
                self.accelerations[:,2] += (1/self.accelerationduration[2])*self.control[2] + numpy.random.normal(0, sdtheta, size=self.PreviousStates.shape[0])
            else:
                self.accelerations[:,2] += (1/self.accelerationduration[2])*deltacontrol[2] + numpy.random.normal(0, sdtheta, size=self.PreviousStates.shape[0])
            self.accelendtimes[:,2] = self.time + self.accelerationduration[2]
        
        self.accelerations = numpy.where(self.accelendtimes > self.time, self.accelerations, 0)
        # calculate the controls contribution to the state velocity
        self.States[:,Localisation.THETADOT] += self.accelerations[:,2]*(self.time - self.previoustime)
        
        self.previouscontrol = self.control
        
    def estimateState(self):
        """ Updates the estimate of the state """
        best = numpy.argmin(self.Weights)
        beststate = self.States[best,:]
        
        #print "Best State:", beststate
        
        cond = (numpy.sum(numpy.fabs(self.States - beststate), axis=1) < 10)
        beststates = numpy.compress(cond, self.States, axis=0)
        bestweights = numpy.compress(cond, self.Weights)
        
        #print "States", self.States
        #print "States within window:", cond
        #print "States close to best", beststates
        #print "Weights close to best", bestweights
        
        #print "Product:", (bestweights*beststates.T).T
        
        self.State = numpy.sum((bestweights*beststates.T).T, axis=0)
        self.State = numpy.sum((self.Weights*self.States.T).T, axis=0)
        print "Estimate:", self.State
        if numpy.isnan(self.State[0]):
            print "FAIL"
        self.__updateAttributesFromState()
        
    def __initParticles(self):
        """ Initialises self.Particles to contain Localisation.NUM_PARTICLES particles around the current measurement """
        #print "Initialising Particles around", self.measurement
        self.States += self.measurement
        # I know for certain that at the beginning the robot is not moving, so all of the velocities should be zero. The Position however should get some noise
        self.States[:,Localisation.THETA] += numpy.random.normal(0, 0.02, size=self.States.shape[0])
        # now swap half of the orientations
        self.States[:,Localisation.THETA] = numpy.where(numpy.random.uniform(0,1, size=self.States.shape[0]) < 0.5, self.States[:, Localisation.THETA], self.States[:, Localisation.THETA] - numpy.pi)
        #print self.States
        
    def __naoToState(self, nao):
        state = numpy.zeros(Localisation.STATE_LENGTH)
        if nao != None:
            #state[Localisation.X] = nao.X
            #state[Localisation.Y] = nao.Y
            state[Localisation.THETA] = nao.Orientation
            #state[Localisation.XDOT] = nao.VX
            #state[Localisation.YDOT] = nao.VY
            state[Localisation.THETADOT] = nao.VOrientation
        
            #print nao.X
            #self.AllX.append(nao.X)
            #self.AllY.append(nao.Y)
            #self.AllTheta.append(nao.Orientation)
            #self.AllXdot.append(nao.VX)
            #self.AllYdot.append(nao.VY)
            #self.AllThetadot.append(nao.VOrientation)
            
            #print "SDs: X", numpy.std(self.AllX), "Y", numpy.std(self.AllY), "Theta", numpy.std(self.AllTheta), "Xdot", numpy.std(self.AllXdot), "Ydot", numpy.std(self.AllYdot), "Thetadot", numpy.std(self.AllThetadot)
        
        return state
        
    def __updateAttributesFromState(self):
        """ I have a bunch of convienent attributes for accessing the state. I need to keep them for backward compatiblity purposes. """
        self.X = self.measurement[0]#self.State[Localisation.X]
        self.Y = self.measurement[1]#self.State[Localisation.Y]
        self.Theta = self.State[Localisation.THETA]
        self.VX = 0#self.State[Localisation.XDOT]
        self.VY = 0#self.State[Localisation.YDOT]
        self.VTheta = self.State[Localisation.THETADOT]
        self.V = 0#numpy.sqrt(self.VX**2 + self.VY**2)
        
    def __gauss(self, x, sigma):
        return (1.0/numpy.sqrt(2*numpy.pi*sigma))*numpy.exp(-(x**2)/(2*sigma**2))
        

if __name__ == '__main__':
    import matplotlib
    matplotlib.use('WXAgg')
    matplotlib.rcParams['toolbar'] = 'None'
    
    import pylab, psyco, wx
    psyco.full()
    
    x = list()
    y = list()
    o = list()
    localisation = Localisation()
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
        #x.append(localisation.State[0])
        #y.append(localisation.State[1])
        #o.append(localisation.State[2])
        loopcount += 1
        if loopcount == 2:
            print "Starting"
            control = numpy.array([200,-0.5,0])
            
        canvas.restore_region(update_plot.background)
        estimateplot.set_data(x,y)
        particleplot.set_data(100*numpy.cos(localisation.States[:,Localisation.THETA]), 100*numpy.sin(localisation.States[:,Localisation.THETA]))
        ax.draw_artist(particleplot)
        #ax.draw_artist(estimateplot)
        canvas.blit(ax.bbox)
        time.sleep(max(0,0.1 - (time.time() - starttime)))
        wx.WakeUpIdle()

    update_plot.background = None
    wx.EVT_IDLE(wx.GetApp(), update_plot)
    pylab.show()
