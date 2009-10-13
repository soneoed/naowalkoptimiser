from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg,\
     FigureManager, NavigationToolbar2WxAgg
     
import wx, numpy, pylab

from matplotlib.figure import Figure
from matplotlib.axes import Subplot

from MCLLocalisation import Localisation

[wxID_LEFTPANEL] = [wx.NewId() for _init_ctrls in range(1)]

class NaoPanel(wx.Panel):
    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Panel.__init__(self, id=wxID_LEFTPANEL, name='NaoPanel',
              parent=prnt, pos=wx.Point(208, 8), size=wx.Size(800, 400),
              style=wx.NO_BORDER | wx.TAB_TRAVERSAL)
        self.SetClientSize(wx.Size(800, 400))
        self.SetBackgroundColour(wx.Colour(0, 0, 255))
        self.Bind(wx.EVT_PAINT, self.OnNaoPanelPaint)

    def __init__(self, parent, id, pos, size, style, name):
        self._init_ctrls(parent)
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
            
        ##left : the left side of the subplots of the figure
##     |      right : the right side of the subplots of the figure
##     |      bottom : the bottom of the subplots of the figure
##     |      top : the top of the subplots of the figure
##     |      wspace : the amount of width reserved for blank space between subplots
##     |      hspace : the amount of height reserved for white space between subplots
##     |      

        self.canvas = FigureCanvasWxAgg(self, -1, self.fig)

        ##now put everything in a sizer
        sizer = wx.BoxSizer(wx.VERTICAL)
        # This way of adding to sizer allows resizing
        sizer.Add(self.canvas, 1, wx.LEFT|wx.TOP|wx.GROW)
        self.SetSizer(sizer)
        self.Fit()
        
        ##now finally create the actual plot
        ##self.axes = self.fig.add_subplot(111)
        self.axes = self.fig.add_axes((0.08,0.08,0.90,0.85))               ##left,bottom,width,height
        self.naohistoryplot = self.axes.plot([0,0],[0,0], 'r', animated=True)
        self.naohistoryx = list()
        self.naohistoryy = list()
        
        self.positionmeasurementplot = self.axes.plot([0,0],[0,0], 'blue', marker='o', markersize=5, linewidth=0, markeredgewidth=0, animated=True)
        self.orientationmeasurementplot = self.axes.plot([0,0],[0,0], 'blue', linewidth=2, animated=True)
        self.shapeplot = self.axes.plot([0,0],[0,0], 'blue', marker='o', markersize=2, linewidth=0, markeredgewidth=0, animated=True)
        self.estimateplot = self.axes.plot([0,0],[0,0], 'red', linewidth=2, animated=True)
        self.particleplot = self.axes.quiver([0,0],[0,0], [1,1], [0.5, -0.5], [1, 1], cmap=pylab.gray(), animated=True)
        
        ##plot formatting
        self.axes.set_title('Nao Image', fontsize='10')
        self.axes.set_xlabel('y (cm)', fontsize='10')
        self.axes.set_ylabel('x (cm)', fontsize='10')
        ticks = numpy.arange(-25, 25 + 5, 5)
        labels = [str(tick) for tick in ticks]
        self.axes.set_yticks(ticks)
        self.axes.set_yticklabels(labels, fontsize=8)
        self.axes.set_ylim(ticks[0], ticks[-1])
        ticks = -numpy.arange(-50, 50+5, 5)
        labels = [str(tick) for tick in ticks]
        self.axes.set_xticks(ticks)
        self.axes.set_xticklabels(labels,fontsize=8)
        self.axes.set_xlim(ticks[0], ticks[-1])
        
        self.canvas.draw()
        self.canvas.gui_repaint()
        
        # save the clean slate background -- everything but the animated line
        # is drawn and saved in the pixel buffer background
        self.background = self.canvas.copy_from_bbox(self.axes.bbox)
        
        #self.leftedgeplot = self.axes.plot([0,0],[0,0], 'orange', marker='o', markersize=4, linewidth=0, animated=True) 
        #self.rightedgeplot = self.axes.plot([0,0],[0,0], 'purple', marker='o', markersize=4, linewidth=0, animated=True)
        
    def setNaoFinder(self, finder):
        """ """
        self.NAOFinder = finder
        
    def setLocalisation(self, localisation):
        """ """
        self.Localisation = localisation
    
    def updateData(self, data):
        """updateData. Updates the data that this panel is displaying.
        """
        # Note the x values are plotted on the y-axis, and the y values are plotted on the x-axis
        naox = self.Localisation.X
        naoy = self.Localisation.Y
        naoorientation = self.Localisation.Orientation
        
        measurednaox = self.NAOFinder.NaoX
        measurednaoy = self.NAOFinder.NaoY
        measurednaoorientation = self.NAOFinder.NaoOrientation
        
        self.positionmeasurementplot[0].set_data([measurednaoy, measurednaoy], [measurednaox, measurednaox])
        self.orientationmeasurementplot[0].set_data([measurednaoy + 10*numpy.sin(measurednaoorientation - numpy.pi), measurednaoy + 10*numpy.sin(measurednaoorientation)], [measurednaox + 10*numpy.cos(measurednaoorientation - numpy.pi), measurednaox + 10*numpy.cos(measurednaoorientation)])
        self.shapeplot[0].set_data(self.NAOFinder.ShapeY, self.NAOFinder.ShapeX)
        self.naohistoryx.append(naox)
        self.naohistoryy.append(naoy)
        if len(self.naohistoryx) > 20:
            del self.naohistoryx[0]
            del self.naohistoryy[0]
        self.naohistoryplot[0].set_data(self.naohistoryy, self.naohistoryx)

        self.estimateplot[0].set_data([naoy, naoy + 10*numpy.sin(self.Localisation.Orientation)], [naox, naox + 10*numpy.cos(self.Localisation.Orientation)])
        #self.particleplot = self.axes.quiver(numpy.array(self.Localisation.States[:,Localisation.Y]), numpy.array(self.Localisation.States[:,Localisation.X]), -numpy.sin(self.Localisation.States[:,Localisation.THETA]), numpy.cos(self.Localisation.States[:,Localisation.THETA]), 1.0 - self.Localisation.GUIWeights, headlength=10, headwidth=10, width=0.001, scale=50.0)

        self.axes.set_xlim(naoy + 50, naoy - 50)
        self.axes.set_ylim(naox - 25, naox + 25)
        # restore the clean slate background
        self.canvas.restore_region(self.background)
        # just draw the animated artist
        self.axes.draw_artist(self.shapeplot[0])
        #self.axes.draw_artist(self.particleplot)
        self.axes.draw_artist(self.naohistoryplot[0])
        self.axes.draw_artist(self.orientationmeasurementplot[0])
        self.axes.draw_artist(self.positionmeasurementplot[0]) 
        self.axes.draw_artist(self.estimateplot[0]) 

        # just redraw the axes rectangle
        self.canvas.blit(self.axes.bbox)
        
        
        #leftx = list()
        #lefty = list()
        #for leftedge in self.NAOFinder.LeftEdges:
        #    leftx.append(data[0][leftedge])
        #    lefty.append(data[1][leftedge])
        
        #rightx = list()
        #righty = list()
        #for rightedge in self.NAOFinder.RightEdges:
        #    rightx.append(data[0][rightedge])
        #    righty.append(data[1][rightedge])
        
        #self.leftedgeplot[0].set_data(lefty, leftx)
        #self.rightedgeplot[0].set_data(righty, rightx)

    def OnNaoPanelPaint(self, event):
        pass
        #self.canvas.draw()
