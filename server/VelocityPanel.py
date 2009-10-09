from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg,\
     FigureManager, NavigationToolbar2WxAgg
     
import wx, numpy, time

from matplotlib.figure import Figure
from matplotlib.axes import Subplot

[wxID_RIGHTPANEL] = [wx.NewId() for _init_ctrls in range(1)]

class VelocityPanel(wx.Panel):
    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Panel.__init__(self, id=wxID_RIGHTPANEL, name='VelocityPanel',
              parent=prnt, pos=wx.Point(8, 416), size=wx.Size(1000, 250),
              style=wx.TAB_TRAVERSAL)
        self.SetClientSize(wx.Size(1000, 250))
        self.SetBackgroundColour(wx.Colour(0, 0, 255))
        self.Bind(wx.EVT_PAINT, self.OnPolarPanelPaint)

    def __init__(self, parent, id, pos, size, style, name):
        self._init_ctrls(parent)
        
        self.Times = list()
        self.VelocitiesX = list()
        self.VelocitiesY = list()
        self.Velocities = list()
        
        self._initFigure()
        
        ##now finally create the actual plot
        self.x_axes = self.fig.add_subplot(131)                 ## the x velocity
        self.y_axes = self.fig.add_subplot(132)                 ## the y velocity
        self.m_axes = self.fig.add_subplot(133)                 ## the magnitude of the velocity
        #self.axes = self.fig.add_axes((0.1,0.1,0.85,0.8))        ##left,bottom,width,height
        
        self.x_plot = self.x_axes.plot([0,1],[0,1],'b', animated=True)
        self.y_plot = self.y_axes.plot([0,1],[0,1],'b', animated=True)
        self.m_plot = self.m_axes.plot([0,1],[0,1],'r', animated=True)
        
        self.x_axes.set_title('X', fontsize='10')
        self.y_axes.set_title('Y', fontsize='10')
        self.m_axes.set_title('M', fontsize='10')
        self.x_axes.set_ylabel('Velocity (cm/s)', fontsize='10')
        #self.axes.set_ylabel('x (cm)', fontsize='10')

        ##plot formatting
        self._formatAxes(self.x_axes)
        self._formatAxes(self.y_axes)
        self._formatAxes(self.m_axes)
        #self.axes.set_title('Velocity', fontsize='10')
        
        self.canvas.draw()
        self.canvas.gui_repaint()
        
        # save the clean slate background -- everything but the animated line
        # is drawn and saved in the pixel buffer background
        self.background = self.canvas.copy_from_bbox(self.fig.bbox)
        
    def _initFigure(self):
        ##Create a matplotlib figure/canvas in this panel
        ##the background colour will be the same as the panel
        ##the size will also be the same as the panel
        ##calculate size in inches
        pixels_width,pixels_height = self.GetSizeTuple()
        self.dpi = 96.0
        inches_width = pixels_width/self.dpi
        inches_height = pixels_height/self.dpi
        
        ##calculate colour in RGB 0 to 1
        colour = self.GetBackgroundColour()
        self.fig = Figure(figsize=(inches_width,inches_height), dpi = self.dpi\
            ,facecolor=(colour.Red()/255.0, colour.Green()/255.0, colour.Blue()/255.0)\
            ,edgecolor=(colour.Red()/255.0, colour.Green()/255.0, colour.Blue()/255.0))    

        self.canvas = FigureCanvasWxAgg(self, -1, self.fig)

        self.fig.subplots_adjust(left=0.05,right=0.95,wspace=0.08)

        ##now put everything in a sizer
        sizer = wx.BoxSizer(wx.VERTICAL)
        # This way of adding to sizer allows resizing
        sizer.Add(self.canvas, 1, wx.LEFT|wx.TOP|wx.GROW)
        self.SetSizer(sizer)
        self.Fit()
        
    def _formatAxes(self, axes):
        """ """
        ticks = numpy.arange(-25, 25+5, 5)
        labels = [str(tick) for tick in ticks]              # velocity
        axes.set_yticks(ticks)
        axes.set_yticklabels(labels, fontsize=8)
        ticks = numpy.arange(0, 10+1.0, 1.0)                # time
        labels = [str(tick) for tick in ticks]              
        axes.set_xticks(ticks)
        axes.set_xticklabels(labels,fontsize=8)
        
        #if axes == self.m_axes:
        #    self.axes.set_xlabel('time (s)', fontsize='10')
        #self.axes.set_ylabel(' (mm)', fontsize='10')
        
        
    
    def updateData(self, velx, vely, vel):
        """updateData. Updates the data that this panel is displaying.
        """
        self.VelocitiesX.append(velx)
        self.VelocitiesY.append(vely)
        self.Velocities.append(vel)
        timenow = time.time()
        self.Times.append(timenow)
        if timenow - self.Times[0] > 10:
            del self.Times[0]
            del self.VelocitiesX[0]
            del self.VelocitiesY[0]
            del self.Velocities[0]
        
        self.x_plot[0].set_data(numpy.array(self.Times) - self.Times[0], self.VelocitiesX)
        self.y_plot[0].set_data(numpy.array(self.Times) - self.Times[0], self.VelocitiesY)
        self.m_plot[0].set_data(numpy.array(self.Times) - self.Times[0], self.Velocities)
        
        # restore the clean slate background
        self.canvas.restore_region(self.background)
        # just draw the animated artist
        self.fig.draw_artist(self.x_plot[0])
        self.fig.draw_artist(self.y_plot[0])
        self.fig.draw_artist(self.m_plot[0])
        # just redraw the axes rectangle
        self.canvas.blit(self.fig.bbox)

    def OnPolarPanelPaint(self, event):
        pass
        ##self.canvas.draw()

