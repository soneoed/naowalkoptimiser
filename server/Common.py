""" Common functions and constants that are used everywhere

by Jason Kulk (jason.555@gmail.com)

Copyright (c) 2005, 2006, 2007, 2008, 2009 Jason Kulk 

Common.py is part of DatalogDyno.

DatalogDyno is free software: you can redistribute it and/or modify
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

import wx,os,sys

SRCDIR = 'Source'            ## the relative path to the source files
IMAGEDIR = 'Images'          ## the relative path to the image files  
SYSDIR = 'Sys'               ## the relative path to the (DatalogDyno) system files 

class FileLocator:
    """Contains functions to locate system files. It has been designed to be platform independant.
    That is it makes use of os.path to create the path to the necessary files"""
    
    @staticmethod
    def getSourcePath(filename):
        """Returns the path to the source file
        Input: the filename (including extension) of the source file to get the path of
        Output: a OS-independant path to the source file"""
        path = os.path.join(SRCDIR, filename)
        return path
    
    @staticmethod
    def getImagePath(filename):
        """Returns the path to the image file
        Input: the filename (including extension) of the image to get the path of
        Output: a OS-independant path to the image file"""
        path = os.path.join(IMAGEDIR, filename)
        return path
    
    @staticmethod
    def getSysPath(filename):
        """Returns the path to the (DatalogDyno) system file
        Input: the filename (including extension) of the sys file to get the path of
        Output: a OS-independant path to the sys file"""
        path = os.path.join(SYSDIR,filename)
        return path
    
class Platform:
    """Contains functions to determine which platform we are running on"""
    
    ##os.name can have the following values
    ##'posix', 'nt', 'dos', 'os2', 'mac', or 'ce'
    
    @staticmethod
    def isMac():
        """Returns true is this is a Mac"""
        if os.name == 'mac' or sys.platform == 'darwin':
            return True
        else:
            return False
        
    @staticmethod
    def isLinux():
        """Returns True if this is Linux"""
        if os.name == 'posix' and os.name != 'mac':
            return True
        else:
            return False
        
    @staticmethod
    def isWindows():
        """Returns True if this is Windows"""
        if os.name == 'nt':
            return True
        else:
            return False
    