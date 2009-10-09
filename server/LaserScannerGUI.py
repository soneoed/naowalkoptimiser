""" LaserScannerGUI.

This file will display the laser range data.

Jason Kulk
jason.555@gmail.com

Copyright (c) 2009 Jason Kulk 

LaserScannerGUI.py is part of Jason's Laser Robot Tracking System.

Jason's Laser Robot Tracking System is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import wx, time

import MainFrame

modules ={'MainFrame': [0, 'Main frame of Application', 'MainFrame.py']}

class LaserScannerGUI(wx.App):
    def OnInit(self):
        wx.InitAllImageHandlers()
        self.main = MainFrame.create(None)
        self.main.Show()
        self.SetTopWindow(self.main)
        self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.Bind(wx.EVT_KEY_UP, self.OnKeyUp)
        return True     
        
    def OnKeyDown(self, event):
        self.main.OnMainKeyDown(event)
        event.Skip()
        
    def OnKeyUp(self, event):
        self.main.OnMainKeyUp(event)
        

        
def main():
    gui = LaserScannerGUI(0)
   
    while gui.main.closed == False:
        gui.main.updateData()
        gui.main.Update()
        wx.Yield()
        #time.sleep(1)

if __name__ == '__main__':
    main()
