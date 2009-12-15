import matplotlib
matplotlib.use('WXAgg')

import wx, time, socket, math, psyco, numpy
from LaserScanner import HokuyoLaserScanner
from NAOFinder import NAOFinder
from MCLLocalisation import Localisation
#from Behaviour import Behaviour
from CartesianPanel import CartesianPanel
from NaoPanel import NaoPanel
from VelocityPanel import VelocityPanel

from Network import Network

def create(parent):
    return MainFrame(parent)

[wxID_MAINFRAME, wxID_MAINFRAMEPANEL1, wxID_MAINFRAMEPANEL2] = [wx.NewId() for _init_ctrls in range(3)]

class MainFrame(wx.Frame):
    
    _custom_classes = {'wx.Panel' : ['CartesianPanel','VelocityPanel']}
    
    def _init_ctrls(self, prnt):
        wx.Frame.__init__(self, id=wxID_MAINFRAME, name='MainFrame',
              parent=prnt, pos=wx.Point(150, 80), size=wx.Size(1016, 674),
              style=wx.DEFAULT_FRAME_STYLE, title='Laser Scanner GUI')
        self.SetClientSize(wx.Size(1016, 674))
        self.SetBackgroundColour(wx.Colour(0, 0, 255))

        self.CartesianPanel = CartesianPanel(id=wxID_MAINFRAMEPANEL1, name='CartesianPanel',
              parent=self, pos=wx.Point(8, 8), size=wx.Size(200, 400),
              style=wx.TAB_TRAVERSAL)
              
        self.NaoPanel = NaoPanel(id=wxID_MAINFRAMEPANEL1, name='NaoPanel',
              parent=self, pos=wx.Point(208, 8), size=wx.Size(800, 400),
              style=wx.TAB_TRAVERSAL)

        self.VelocityPanel = VelocityPanel(id=wxID_MAINFRAMEPANEL2, name='VelocityPanel',
              parent=self, pos=wx.Point(8, 416), size=wx.Size(1008, 250),
              style=wx.TAB_TRAVERSAL)
              
        self.Bind(wx.EVT_CLOSE, self.OnMainFrameClose)
        
        self.NumUpdates = 0
        self.StartTime = time.time()

    def __init__(self, parent):
        self._init_ctrls(parent)
        psyco.full()
        self.laserscanner = HokuyoLaserScanner()
        self.localisation = Localisation(400)
        self.naofinder = NAOFinder(self.localisation)
        self.network = Network()
        
        ## control variables
        self.control = numpy.zeros(3)
        # automatic
        self.state = 'init'         # states: init, chase, position, lost
        self.statecount = 0
        self.targetnumber = 0
        #self.targets = [[250, -100], [250, 100], [50, 100], [50, -100]]
        #self.targets = [[275, -10], [275, 10], [50, 10], [50, -10]]
        self.targets = [[275, 0], [50, 0]]

        # manual
        self.up = False             # set these to true when the key is pressed!
        self.down = False
        self.left = False
        self.right = False
        
        self.distance = 0
        self.bearing = 0
        self.orientation = 0
        
        self.CartesianPanel.setNaoFinder(self.naofinder)
        self.NaoPanel.setNaoFinder(self.naofinder)
        self.NaoPanel.setLocalisation(self.localisation)
        self.closed = False
        
    def updateData(self):
        if self.closed == False:
            polardata, cartesiandata = self.laserscanner.getRangeData()
            self.measurement = self.naofinder.findNAO(polardata, cartesiandata)
            self.localisation.update(self.control, self.measurement)
            
            if self.keyPressed() or self.distance != 0 or self.bearing != 0:
                self.calculateWalkControl()
            else:
                self.automaticWalk()
            
            self.network.send(self.localisation.VX, self.localisation.VY, self.localisation.V, self.control[0], self.control[1], self.control[2])
            
            self.CartesianPanel.updateData(cartesiandata, self.localisation.X, self.localisation.Y)
            self.NaoPanel.updateData(cartesiandata)
            self.VelocityPanel.updateData(self.localisation.VX, self.localisation.VY, self.localisation.V)
            self.NumUpdates = self.NumUpdates + 1
            #print self.NumUpdates/(time.time() - self.StartTime)
            
    
    def automaticWalk(self):
        """ """
        # do state transitions if close then then switch to away, if far switch to toward
        if self.state == 'init':
            self.statecount += 1
            if self.statecount > 20:
                print "automaticWalk: init->chase"
                self.state = 'chase'
                self.statecount = 0
        elif self._robotLost():
            self.state = 'lost'
        elif self.state == 'chase':
            self.statecount += 1
            if self.statecount > 30 and self._limitReached():
                print "automaticWalk: chase->position"
                self.state = 'position'
                self.targetnumber = (self.targetnumber + 1)%len(self.targets)
                self.statecount = 0
        elif self.state == 'position':
            self.statecount += 1
            if self.statecount > 5 and self._facingTarget():
                print "automaticWalk: position->chase"
                self.state = 'chase'
                self.statecount = 0
        elif self.state == 'lost':
            if self._robotLost() == False:
                self.state = 'position'
            

        if self.state == 'init' or self.state == 'chase':
            self.control = [-1, 0, 0]
        elif self.state == 'position':
            x, y = self.calculateRelativeTarget(self.targets[self.targetnumber])
            bearing = numpy.arctan2(y,x)
            self.control = [-1, bearing, 0]
        elif self.state == 'lost':
            self.control = [0, 0, 0]
        else:
            print "automaticWalk: ERROR. Unknown state", self.state
            
        self.control = numpy.array(self.control)
        if abs(self.control[1]) > 2.59:
            self.control[1] = (self.control[1]/abs(self.control[1]))*2.59
        
    def _limitReached(self):
        """ Returns true if the robot has reached the border of its play area """
        innerlimit = 50
        sidelimit = 150
        outerlimit = 275
        
        if self.localisation.X < innerlimit:
            return True
        if math.fabs(self.localisation.Y) > sidelimit:
            return True
        if numpy.sqrt(self.localisation.X**2 + self.localisation.Y**2) > outerlimit:
            return True
            
        return False
            
    def _targetReached(self):
        """ Returns true if the current target (self.targets[self.targetnumber]) has been reached"""
        target = self.targets[self.targetnumber]
        
        if numpy.sqrt((target[0] - self.localisation.X)**2 + (target[1] - self.localisation.Y)**2) < 20:
            return True
        else:
            return False
            
    def _facingTarget(self):
        """ Returns true if we are facing the target """
        target = self.targets[self.targetnumber]
        x, y = self.calculateRelativeTarget(target)
        bearing = numpy.arctan2(y,x)
        if math.fabs(bearing) < 0.25:
            return True
        else:
            return False
            
    def _robotLost(self):
        """ Returns true if the robot has gone beyond the border of its play area """
        if self.localisation.X < 20:
            return True
        if numpy.sqrt(self.localisation.X**2 + self.localisation.Y**2) > 350:
            return True
            
        return False
            
    def calculateRelativeTarget(self, target):
        """ """
        x = target[0]
        y = target[1]
        relativeX = (x - self.localisation.X)*math.cos(self.localisation.Orientation) + (y - self.localisation.Y)*math.sin(self.localisation.Orientation)
        relativeY = -(x - self.localisation.X)*math.sin(self.localisation.Orientation) + (y - self.localisation.Y)*math.cos(self.localisation.Orientation)
        
        return [relativeX, relativeY]
            
            
    def calculateWalkControl(self):
    
        deltadistance = 4
        deltabearing = 0.1
        
        if self.up == True:
            if self.distance < 0:
                self.distance = 0
            self.distance += deltadistance
            if self.distance > 100:
                self.distance = 100
        elif self.down == True:
            if self.distance > 0:
                self.distance = 0
            self.distance -= deltadistance
            if self.distance < -30:
                self.distance = -30
        else:
            if self.distance > 0:
                self.distance -= deltadistance
                if self.distance < 0:
                    self.distance = 0
            elif self.distance < 0:
                self.distance += deltadistance
                if self.distance > 0:
                    self.distance = 0
            
        if self.left == True:
            if self.bearing < 0:
                self.bearing += 3*deltabearing
            else:
                self.bearing += deltabearing

            if self.bearing > 3:
                self.bearing = 3;
        elif self.right == True:
            if self.bearing > 0:
                self.bearing = 0

            self.bearing -= deltabearing

            if self.bearing < -3:
                self.bearing = -3;
        else:
            if self.bearing > 0:
                self.bearing -= deltabearing
                if self.bearing < 0:
                    self.bearing = 0
            elif self.bearing < 0:
                self.bearing += deltabearing
                if self.bearing > 0:
                    self.bearing = 0
            
        if self.distance == 0:
            # then I want to turn on the spot
            self.control = [0, 0, self.bearing]
        elif self.distance < 0:
            ## then I want to walk backwards
            self.control = [-1*self.distance, math.pi, 0]
        else:
            self.control = [-1, self.bearing, 1001]
            
        self.control = numpy.array(self.control)
            
    def keyPressed(self):
        """ Returns True if we are manually controlling the robot """
        return self.up or self.down or self.left or self.right
        
            
    def OnMainKeyDown(self, event):
        
        code = event.GetKeyCode()
        if code == wx.WXK_UP:
            self.up = True
        elif code == wx.WXK_DOWN:
            self.down = True
        elif code == wx.WXK_LEFT:
            self.left = True
        elif code == wx.WXK_RIGHT:
            self.right = True
            
        event.Skip()
    
    
    def OnMainKeyUp(self, event):
        
        code = event.GetKeyCode()
        if code == wx.WXK_UP:
            self.up = False
        elif code == wx.WXK_DOWN:
            self.down = False
        elif code == wx.WXK_LEFT:
            self.left = False
        elif code == wx.WXK_RIGHT:
            self.right = False
        
        event.Skip()
        
        
    def OnMainFrameClose(self, event):
        self.laserscanner.stop()
        self.closed = True
        event.Skip()
        
