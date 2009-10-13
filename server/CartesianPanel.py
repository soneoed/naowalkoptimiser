from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg,\
     FigureManager, NavigationToolbar2WxAgg
     
import wx, numpy

from matplotlib.figure import Figure
from matplotlib.axes import Subplot

[wxID_LEFTPANEL] = [wx.NewId() for _init_ctrls in range(1)]

class CartesianPanel(wx.Panel):
    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Panel.__init__(self, id=wxID_LEFTPANEL, name='CartesianPanel',
              parent=prnt, pos=wx.Point(8, 8), size=wx.Size(200, 400),
              style=wx.NO_BORDER | wx.TAB_TRAVERSAL)
        self.SetClientSize(wx.Size(200, 400))
        self.SetBackgroundColour(wx.Colour(0, 0, 255))
        self.Bind(wx.EVT_PAINT, self.OnCartesianPanelPaint)

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
        self.axes = self.fig.add_axes((0.16,0.08,0.90,0.85))               ##left,bottom,width,height
        self.plot = self.axes.plot([0,0],[0,0], 'b', animated=True)
        self.naohistoryplot = self.axes.plot([0,0],[0,0], 'r', animated=True)
        self.naohistoryx = list()
        self.naohistoryy = list()
        self.naoplot = self.axes.plot([0,0],[0,0], 'r', marker='o', markersize=4, animated=True)
        
        self.leftedgeplot = self.axes.plot([0,0],[0,0], 'orange', marker='o', markersize=4, linewidth=0, animated=True) 
        self.rightedgeplot = self.axes.plot([0,0],[0,0], 'purple', marker='o', markersize=4, linewidth=0, animated=True) 

        ##plot formatting
        self.axes.set_title('Laser Image', fontsize='10')
        #self.axes.set_xlabel('y (cm)', fontsize='10')
        #self.axes.set_ylabel('x (cm)', fontsize='10')
        ticks = numpy.arange(-450, 450+100, 100)
        labels = [str(tick) for tick in ticks]
        self.axes.set_yticks(ticks)
        self.axes.set_yticklabels(labels, fontsize=8)
        self.axes.set_ylim(ticks[0], ticks[-1])
        ticks = numpy.arange(0, 450+100, 100)
        labels = [str(tick) for tick in ticks]
        self.axes.set_xticks(ticks)
        self.axes.set_xticklabels(labels,fontsize=8)
        self.axes.set_xlim(ticks[0], ticks[-1])
        
        self.canvas.draw()
        self.canvas.gui_repaint()
        
        # save the clean slate background -- everything but the animated line
        # is drawn and saved in the pixel buffer background
        self.background = self.canvas.copy_from_bbox(self.axes.bbox)
        
    def setNaoFinder(self, finder):
        """ """
        self.NAOFinder = finder
    
    def updateData(self, data, naox, naoy):
        """updateData. Updates the data that this panel is displaying.
        """
        self.x = data[0]
        self.y = data[1]
        
        self.plot[0].set_data(self.x, self.y)
        self.naoplot[0].set_data([naox,naox], [naoy, naoy])
        self.naohistoryx.append(naox)
        self.naohistoryy.append(naoy)
        if len(self.naohistoryx) > 400:
            del self.naohistoryx[0]
            del self.naohistoryy[0]
        self.naohistoryplot[0].set_data(self.naohistoryx, self.naohistoryy)
        
        leftx = list()
        lefty = list()
        for leftedge in self.NAOFinder.LeftEdges:
            leftx.append(data[0][leftedge])
            lefty.append(data[1][leftedge])
        
        rightx = list()
        righty = list()
        for rightedge in self.NAOFinder.RightEdges:
            rightx.append(data[0][rightedge])
            righty.append(data[1][rightedge])
        
        self.leftedgeplot[0].set_data(leftx, lefty)
        self.rightedgeplot[0].set_data(rightx, righty)
                
        # restore the clean slate background
        self.canvas.restore_region(self.background)
        # just draw the animated artist
        self.axes.draw_artist(self.plot[0])
        self.axes.draw_artist(self.naoplot[0])
        self.axes.draw_artist(self.naohistoryplot[0])
        
        self.axes.draw_artist(self.leftedgeplot[0])
        self.axes.draw_artist(self.rightedgeplot[0])

        # just redraw the axes rectangle
        self.canvas.blit(self.axes.bbox)

    def OnCartesianPanelPaint(self, event):
        pass
        #self.canvas.draw()
