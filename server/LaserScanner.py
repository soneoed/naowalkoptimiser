""" Module to communicate with a Hokuyo Laser Scanner

Jason Kulk
"""

import serial
import glob                         ## for unix filepath expansion
import math, time, sys, os, numpy
from Common import Platform         ## existing code to determine whether windows, mac, or linux

class HokuyoLaserScanner:
    def __init__(self, baudrate=115200):
        """ Initialisation requires the serialport to access.
        """
        print "HokuyoLaserScanner. Initialising Laser Scanner"
        
        self._initSerial(baudrate)          ## initialises the serial connection
        self._initScanData()                ## initialises the scan data
        self._initSensor()                  ## initialises the laser scanner itself
        
        print "HokuyoLaserScanner. Initialised Laser Scanner"
        
    def _initSerial(self, m_baudrate):
        self.PortName = self.__getSerialPortName()
        if self.PortName != None:
            self.Serial = serial.Serial(port=self.PortName, baudrate=m_baudrate, timeout=0.15)       ## I updated the firmware, the default baud rate is now 115200
        else:
            self.Serial = None
        
    def __getSerialPortName(self):
        """ Returns the serial port name to be used with serial.Serial().
        Attempts to find the port on linux and mac are automatic, provided there is only one ACM/usbserial device
        The portname is hardcoded for windows """
        if Platform.isMac():
            possibledevices = glob.glob('/dev/tty.usb*')
            if len(possibledevices) == 0:
                print "HokuyoLaserScanner. No candiates for serial connection to laser scanner found"
                portname = None
            elif len(possibledevices) == 1:
                print "HokuyoLaserScanner. Opening port", possibledevices[0]
                portname = possibledevices[0]
            else:
                print "HokuyoLaserScanner. Multiple candiates found:", possibledevices
                portname = possibledevices[0]
        elif Platform.isLinux():
            possibledevices = glob.glob('/dev/ttyACM*')
            if len(possibledevices) == 0:
                print "HokuyoLaserScanner. No candiates for serial connection to laser scanner found"
                portname = None
            elif len(possibledevices) == 1:
                print "HokuyoLaserScanner. Opening port", possibledevices[0]
                portname = possibledevices[0]
            else:
                print "HokuyoLaserScanner. Multiple candiates found:", possibledevices
                portname = possibledevices[0]
        else:
            portname='COM9'
        
        return portname
        
    def _initScanData(self):
        """ Generates self.Angles, and sets the default start and stop points
        """
        URG_04LX_FIRST = 0                  ## the first measurement
        URG_04LX_FIRST_MEASUREMENT = 44     ## the first measurement in the detection range
        URG_04LX_LAST_MEASUREMENT = 725     ## the last measurement in the detection range
        URG_04LX_LAST = 768                 ## the last measurement
        
        self.CentrePoint = int(URG_04LX_LAST/2)                 ## corresponds to zero (straight ahead)
        
        self.StartPoint = self.CentrePoint - 256#URG_04LX_FIRST_MEASUREMENT            ## the 'default' start point
        self.StopPoint = self.CentrePoint + 256#URG_04LX_LAST_MEASUREMENT              ## the 'default' end point
        
        if self.StartPoint < URG_04LX_FIRST_MEASUREMENT:
            self.StartPoint = URG_04LX_FIRST_MEASUREMENT
        if self.StopPoint > URG_04LX_LAST_MEASUREMENT:
            self.StopPoint = URG_04LX_LAST_MEASUREMENT
            
        pointtoangle = 2*math.pi/1024
        self.Angles = numpy.zeros(URG_04LX_LAST-URG_04LX_FIRST)
        for i in range(URG_04LX_FIRST, URG_04LX_LAST):
            self.Angles[i] = (i - self.CentrePoint)* pointtoangle
        
    def _initSensor(self):
        """ """
        self.Mode = 'single'
        print "HokuyoLaserScanner. _initSensor. Initialising Sensor.", 
        if self.Serial == None:
            return
        self.Serial.write('QT\n') 
        if self.Mode == 'continuous':
            clustercount = 0
            scaninterval = 0
            numscans = 0        ## 0 means the scans never stop!
            self.Serial.write('MS%04d%04d%02d%01d%02d\n' % (self.StartPoint, self.StopPoint, clustercount, scaninterval, numscans))
            print "Response:", self.Serial.readlines()
        elif self.Mode == 'single':
            self.Serial.write('BM\n')     
            print "Response:", self.Serial.readlines()
        else:
            print "HokuyoLaserScanner. _initSensor. ERROR Unknown sensor mode:", self.Mode
            sys.exit(1)
        

    def GetVersion(self):
        """ Send 'V' (0x56) and LF """
        pass
        
    def SetLaserIllumination(self, on):
        pass
    
    def _setBaudRate(self, baudrate):
        """ I can't seem to set the baudrate of the cdc acm driver!"""
        if self.Serial == None:
            return
        self.Serial.write('S%06d%07d\n' % (baudrate,0))
        self.Serial.close()
        self.Serial = serial.Serial(port=self.serialdevice, baudrate=self.baudrate, timeout=0.5)
    
    def getRangeData(self):
        result = []
        if self.Mode == 'continuous':
            pass
        elif self.Mode == 'single':
            rangedata = self._getSingleScan()
        angledata = self.Angles[self.StartPoint:self.StopPoint+1]
        try:
            xdata = [distance*math.cos(angle) for distance, angle in zip(rangedata, angledata)]
            ydata = [distance*math.sin(angle) for distance, angle in zip(rangedata, angledata)]
        except:
            print len(rangedata), rangedata
            print len(angledata), angledata
        
        return [[rangedata, angledata], [xdata, ydata]]
        
    def _getSingleScan(self):
        """ """
        if self.Serial == None:
            # A small hack to ensure the rest of the program does not crash when there is no laser scanner attached ;)
            return 409.6*numpy.ones(len(self.Angles[self.StartPoint:self.StopPoint+1]))
        ## first request new data
        command = 'GS%04d%04d%02d\n' % (self.StartPoint,self.StopPoint,0)
        self.Serial.write(command)
        
        ## now read back the reply
        commandlength = len(command)
        statuslength = 4
        timestamplength = 6
        numdatabytes = (self.StopPoint+1 - self.StartPoint)*2
        datalength =  numdatabytes + int(math.ceil(numdatabytes/64.0))*2 + 1
        replylength = commandlength + statuslength + timestamplength + datalength
        
        result = self.Serial.read(size = replylength)
        result = result.split('\n')
        
        ## Reply format:
        ##      - first line is the command echo
        ##      - second line is the status packet [ status[0], status[1], sum[0] ]
        ##      - third line is the timestamp [ time[msb], time[2], time[1], time[lsb], sum[0] ]
        ##      - following lines are the data [ 64 bytes of data, sum[0] ]
        
        commandpacket, statuspacket = result[0], result[1]
        if (statuspacket[0] != '0' or statuspacket[1] != '0'):
            print "HokuyoLaserScanner. _getSingleScan: There was an error with the scanner: ", statuspacket
            return 409.6*numpy.ones(len(self.Angles[self.StartPoint:self.StopPoint+1]))
        else:
            timestamp = result[2]
            data = [line.strip()[0:-1] for line in result[3:]]
            data = ''.join(data)
            integers = iter([ord(char)-0x30 for char in data])
            rangedata = []
            while True:
                try: 
                    distance = ((integers.next() << 6) + integers.next())/10.0      ## the range in cm
                    if distance < 20:
                        distance = 409.6
                    rangedata.append(distance)
                except StopIteration:
                    break
            return rangedata
        
    
    def stop(self):
        """ Closes the serial port """
        if self.Serial == None:
            return
        self.Serial.write('QT\n') 
        self.Serial.close()
        
