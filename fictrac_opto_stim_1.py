# -*- coding: utf-8 -*-
"""
Created on Thu Dec 14 16:39:58 2023

@author: Nikon
"""




import pyfirmata2
from pyfirmata2 import Arduino
import time

board= Arduino('COM9')
time.sleep(2)
print(board.get_firmata_version())


import logging
from mecom import MeCom, ResponseException, WrongChecksum
from serial import SerialException

import numpy as np
#import keyboard
import threading
import time
#import win32gui

import socket
import select

HOST = '127.0.0.1'  # The (receiving) host IP address (sock_host)
PORT = 1123         # The (receiving) host port (sock_port)

posx=None
t=None
h=None
pt=None


#import os
#import time
import math


from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
from sensirion_shdlc_sensorbridge import SensorBridgePort, \
    SensorBridgeShdlcDevice, SensorBridgeI2cProxy
from sensirion_i2c_driver import I2cConnection
from sensirion_i2c_sht.sht4x import Sht4xI2cDevice
from matplotlib import pyplot as plt

port=ShdlcSerialPort(port='COM4', baudrate=460800)
bridge = SensorBridgeShdlcDevice(ShdlcConnection(port), slave_address=0)
print("SensorBridge SN: {}".format(bridge.get_serial_number()))

    # Configure SensorBridge port 1 for SHT3x
bridge.set_i2c_frequency(SensorBridgePort.ONE, frequency=100e3)
bridge.set_supply_voltage(SensorBridgePort.ONE, voltage=3.3)
bridge.switch_supply_on(SensorBridgePort.ONE)

    # Create SHT3x device
i2c_transceiver = SensorBridgeI2cProxy(bridge, port=SensorBridgePort.ONE)
sht4x = Sht4xI2cDevice(I2cConnection(i2c_transceiver))
temperature, humidity = sht4x.single_shot_measurement()
t="{}".format(temperature)
h="{}".format(humidity)
print(h,t)

temperature, humidity = sht4x.single_shot_measurement()
t="{}".format(temperature)
t=float(t[:4])
h="{}".format(humidity)
h=float(h[:4])



import propar
el_flow = propar.instrument('COM10')
humid = propar.instrument('COM10',14)
dry = propar.instrument('COM10',3)

# default queries from command table below
DEFAULT_QUERIES = [
    "loop status",
    "object temperature",
    "target object temperature",
    "output current",
    "output voltage"
]

# syntax
# { display_name: [parameter_id, unit], }
COMMAND_TABLE = {
    "loop status": [1200, ""],
    "object temperature": [1000, "degC"],
    "target object temperature": [1010, "degC"],
    "output current": [1020, "A"],
    "output voltage": [1021, "V"],
    "sink temperature": [1001, "degC"],
    "ramp temperature": [1011, "degC"],
}


class MeerstetterTEC(object):
    """
    Controlling TEC devices via serial.
    """

    def _tearDown(self):
        self.session().stop()

    def __init__(self, port="COM12", channel=1, queries=DEFAULT_QUERIES, *args, **kwars):
        assert channel in (1, 2)
        self.channel = channel
        self.port = port
        self.queries = queries
        self._session = None
        self._connect()

    def _connect(self):
        # open session
        self._session = MeCom(serialport=self.port)
        # get device address
        self.address = self._session.identify()
        logging.info("connected to {}".format(self.address))

    def session(self):
        if self._session is None:
            self._connect()
        return self._session

    def get_data(self):
        data = {}
        for description in self.queries:
            id, unit = COMMAND_TABLE[description]
            try:
                value = self.session().get_parameter(parameter_id=id, address=self.address, parameter_instance=self.channel)
                data.update({description: (value, unit)})
            except (ResponseException, WrongChecksum) as ex:
                self.session().stop()
                self._session = None
        return data

    def set_temp(self, value):
        """
        Set object temperature of channel to desired value.
        :param value: float
        :param channel: int
        :return:
        """
        # assertion to explicitly enter floats
        assert type(value) is float
        logging.info("set object temperature for channel {} to {} C".format(self.channel, value))
        return self.session().set_parameter(parameter_id=3000, value=value, address=self.address, parameter_instance=self.channel)

    def _set_enable(self, enable=True):
        """
        Enable or disable control loop
        :param enable: bool
        :param channel: int
        :return:
        """
        value, description = (1, "on") if enable else (0, "off")
        logging.info("set loop for channel {} to {}".format(self.channel, description))
        return self.session().set_parameter(value=value, parameter_name="Status", address=self.address, parameter_instance=self.channel)

    def enable(self):
        return self._set_enable(True)

    def disable(self):
        return self._set_enable(False)

mc = MeerstetterTEC()
print(mc.get_data())

board.digital[2].write(0)


global posx
posx=None
t=None
h=None
pt=None


def valve_op_calc(max_dist, actual_dist):
    a=15000   ### put a scalebar to adjust this distance !!!
    
    old_max=2*max_dist
    old_min=0
    
    new_max=7500
    new_min=-7500
    old_range=old_max-old_min
    new_range=new_max-new_min
    
    old_val=actual_dist
    if old_val>old_max:
        old_val=old_val%old_max
    new_val=(((old_val-old_min)*new_range)/old_range)+new_min
    if actual_dist>old_max:
        div=int(old_val/old_max)
        if div%2!=0:
            new_val=new_val*(-1)
    if new_val<0:
        new_val=new_val*(-1)
    new_val=new_val*2
    return(new_val)








from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1305, 797)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(130, 170, 231, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(150, 240, 181, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(460, 170, 231, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setGeometry(QtCore.QRect(500, 240, 141, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setObjectName("pushButton_4")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(270, 470, 321, 31))
        self.lineEdit.setObjectName("lineEdit")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(170, 470, 91, 21))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.spinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox.setGeometry(QtCore.QRect(960, 230, 91, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.spinBox.setFont(font)
        self.spinBox.setMinimum(15)
        self.spinBox.setMaximum(40)
        self.spinBox.setObjectName("spinBox")
        self.lcdNumber = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdNumber.setGeometry(QtCore.QRect(930, 390, 81, 41))
        self.lcdNumber.setSmallDecimalPoint(False)
        self.lcdNumber.setDigitCount(5)
        self.lcdNumber.setSegmentStyle(QtWidgets.QLCDNumber.Filled)
        self.lcdNumber.setObjectName("lcdNumber")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(730, 390, 191, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.horizontalSlider = QtWidgets.QSlider(self.centralwidget)
        self.horizontalSlider.setGeometry(QtCore.QRect(960, 150, 311, 71))
        self.horizontalSlider.setMaximum(15000)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.horizontalSlider.setTickInterval(200)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.lcdNumber_2 = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdNumber_2.setGeometry(QtCore.QRect(1200, 390, 81, 41))
        self.lcdNumber_2.setObjectName("lcdNumber_2")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(1080, 390, 111, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(610, 0, 301, 61))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setItalic(True)
        font.setUnderline(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_5.setGeometry(QtCore.QRect(770, 170, 171, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_5.setFont(font)
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_6.setGeometry(QtCore.QRect(790, 230, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_6.setFont(font)
        self.pushButton_6.setObjectName("pushButton_6")
        self.radioButton = QtWidgets.QRadioButton(self.centralwidget)
        self.radioButton.setGeometry(QtCore.QRect(1000, 340, 151, 21))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.radioButton.setFont(font)
        self.radioButton.setObjectName("radioButton")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(1110, 460, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.lcdNumber_3 = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdNumber_3.setGeometry(QtCore.QRect(1200, 460, 81, 41))
        self.lcdNumber_3.setObjectName("lcdNumber_3")
        self.spinBox_2 = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox_2.setGeometry(QtCore.QRect(1120, 100, 151, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.spinBox_2.setFont(font)
        self.spinBox_2.setMaximum(30000)
        self.spinBox_2.setObjectName("spinBox_2")
        self.pushButton_7 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_7.setGeometry(QtCore.QRect(970, 100, 141, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_7.setFont(font)
        self.pushButton_7.setObjectName("pushButton_7")
        self.checkBox = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox.setGeometry(QtCore.QRect(970, 50, 161, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.checkBox.setFont(font)
        self.checkBox.setObjectName("checkBox")
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.doubleSpinBox.setGeometry(QtCore.QRect(160, 620, 81, 31))
        self.doubleSpinBox.setMaximum(150000.0)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.doubleSpinBox_2 = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.doubleSpinBox_2.setGeometry(QtCore.QRect(160, 660, 81, 31))
        self.doubleSpinBox_2.setMaximum(150000.0)
        self.doubleSpinBox_2.setObjectName("doubleSpinBox_2")
        self.doubleSpinBox_3 = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.doubleSpinBox_3.setGeometry(QtCore.QRect(160, 700, 81, 31))
        self.doubleSpinBox_3.setMaximum(150000.0)
        self.doubleSpinBox_3.setObjectName("doubleSpinBox_3")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(100, 620, 61, 31))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(100, 700, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setGeometry(QtCore.QRect(100, 660, 61, 21))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.doubleSpinBox_4 = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.doubleSpinBox_4.setGeometry(QtCore.QRect(470, 620, 81, 31))
        self.doubleSpinBox_4.setObjectName("doubleSpinBox_4")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(300, 610, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.pushButton_8 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_8.setGeometry(QtCore.QRect(610, 620, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_8.setFont(font)
        self.pushButton_8.setObjectName("pushButton_8")
        self.pushButton_9 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_9.setGeometry(QtCore.QRect(740, 620, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.pushButton_9.setFont(font)
        self.pushButton_9.setObjectName("pushButton_9")
        self.checkBox_2 = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox_2.setGeometry(QtCore.QRect(140, 360, 211, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.checkBox_2.setFont(font)
        self.checkBox_2.setObjectName("checkBox_2")
        self.spinBox_3 = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox_3.setGeometry(QtCore.QRect(570, 360, 71, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.spinBox_3.setFont(font)
        self.spinBox_3.setObjectName("spinBox_3")
        self.spinBox_3.setMaximum(30000)
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(370, 360, 201, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_10.setFont(font)
        self.label_10.setObjectName("label_10")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1305, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        
        self.pushButton.clicked.connect(self.start_read_data)
        self.pushButton_2.clicked.connect(self.start_stimulus)
        self.pushButton_3.clicked.connect(self.stop_reading_data)
        self.pushButton_4.clicked.connect(self.stop_stimulus)
        self.pushButton_5.clicked.connect(self.set_valve_opening)
        self.pushButton_6.clicked.connect(self.set_temp)
        self.radioButton.toggled.connect(self.start_display_value)
        self.pushButton_7.clicked.connect(self.set_valve)
        self.pushButton_8.clicked.connect(self.start_pid)
        self.pushButton_9.clicked.connect(self.stop_pid)
        self.checkBox_2.toggled.connect(self.start_opto_stim)
         
         
         
         
         ######## flags
        
        self.read_flag=False
        self.stim_flag=False
    
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "read fictrac data"))
        self.pushButton_3.setText(_translate("MainWindow", "stop reading data"))
        self.pushButton_2.setText(_translate("MainWindow", "start stimulus"))
        self.pushButton_4.setText(_translate("MainWindow", "stop stimulus"))
        self.label.setText(_translate("MainWindow", "save data"))
        self.label_3.setText(_translate("MainWindow", "temp of peltier plate "))
        self.label_5.setText(_translate("MainWindow", "fly humidity"))
        self.label_6.setText(_translate("MainWindow", "FICTRAC STIMULUS"))
        self.pushButton_5.setText(_translate("MainWindow", "set valve opening"))
        self.pushButton_6.setText(_translate("MainWindow", "set temperature"))
        self.radioButton.setText(_translate("MainWindow", "display value"))
        self.label_2.setText(_translate("MainWindow", "fly temp"))
        self.pushButton_7.setText(_translate("MainWindow", "valve setting"))
        self.checkBox.setText(_translate("MainWindow", "check valve"))
        self.label_4.setText(_translate("MainWindow", "kp"))
        self.label_7.setText(_translate("MainWindow", "kd"))
        self.label_8.setText(_translate("MainWindow", "ki"))
        self.label_9.setText(_translate("MainWindow", "humidity setpoint"))
        self.pushButton_8.setText(_translate("MainWindow", "start pid "))
        self.pushButton_9.setText(_translate("MainWindow", "stop pid "))
        self.checkBox_2.setText(_translate("MainWindow", "Include optogenetics"))
        self.label_10.setText(_translate("MainWindow", "start time delay in sec"))
        self.lineEdit.setText(_translate("MainWindow", "C:/fictrac/data/dessicated/ir40a/"))
    
    def read_data(self):
        
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            
            sock.bind((HOST, PORT))
            sock.setblocking(True)
            
            # Keep receiving data until FicTrac closes
            data = ""
            timeout_in_seconds = 1
            while self.read_flag==True:
                # Check to see whether there is data waiting
                ready = select.select([sock], [], [], timeout_in_seconds)
                
                # Only try to receive data if there is data waiting
                if ready[0]:
                    # Receive one data frame
                    new_data = sock.recv(1024)
                    
                    
                    if not new_data:
                        break
                    
                    # Decode received data
                    data += new_data.decode('UTF-8')
                    
                    # Find the first frame of data
                    endline = data.find("\n")
                    line = data[:endline]       # copy first frame
                    data = data[endline+1:]     # delete first frame
                    
                    # Tokenise
                    toks = line.split(", ")
                    
                    # Check that we have sensible tokens
                    if ((len(toks) < 24) | (toks[0] != "FT")):
                        print('Bad read')
                        continue
                    
                    
                    global posx
                    global posy
                    global cnt
                    global speed
                    posx = float(toks[15])
                    posy = float(toks[16])
                    cnt = float(toks[1])
                    speed = float(toks[19])
                   # print(posx,posy)
                
                
                else:
                    print('retrying')
            # Didn't find any data - try again
    
    
    def start_read_data(self):
        self.read_flag=True
        threading.Thread(target=self.read_data).start()
    
    def stop_reading_data(self):
        self.read_flag=False
    
    
    def stimulus(self):
        global f
        f= open(str(self.lineEdit.text())+'.txt', 'w')
        while self.stim_flag==True:
            if not posx:
                print('no value detected')
                time.sleep(1)
                ta1=time.time()
            
            else:
                x=posx*4.5
                y=posy*4.5
                z=cnt
                ta2=time.time()-ta1
                #print(z)
                ################################
                dist=((x**2)+(y**2))**0.5
                max_dist=50 ############# put a scalebar in the gui to sdjust this value !!!
                
                if self.checkBox.isChecked()==True:
                    valve_op=self.spinBox_2.value()
                
                else:    
                    valve_op=valve_op_calc(max_dist, dist)
                ###################################
                # if x==0:
                #     x=x+0.001
                # if y==0:
                #     y=0+0.001    
                
                # ang=math.degrees(math.atan(abs(y)/abs(x)))
                # if x>0 and y>0:
                #     ang=ang
                #     #print ('first quadrent', ang)
                
                # if x<0 and y>0:
                #     ang=180-ang
                #     #print('second quadrent',ang)
                
                # if x<0 and y<0:
                #     ang=180+ang
                #     #print('third quadrent',ang)
                # if x>0 and y<0:
                #     ang=360-ang
                #     #print('fourth quadrent', ang)
                
                
                humid.writeParameter(9, int(valve_op))
                dry.writeParameter(9, int(16000)-int(valve_op))
                f.write(str(x) + ','+ str(y)+ ',' + str(dist)+','  +  str(speed)+ ',' + str(z) + ',' +str(t[:4]) + ',' + str(h[:4]) +',' +str(ta2))
                f.write('\n')
                print(dist,ta2)
                time.sleep(0.5)
    
    def start_stimulus(self):
        self.stim_flag=True
        threading.Thread(target=self.stimulus).start()
    
    def stop_stimulus(self):
        f.close()
        posx=None
        t=None
        h=None
        pt=None
        self.stim_flag=False
    
    
    def set_valve_opening(self):
        x=self.horizontalSlider.value()
        dry.writeParameter(9, int(x))
        humid.writeParameter(9, 15000-int(x))
    
    def set_valve(self):
        x=self.spinBox_2.value()
        dry.writeParameter(9, int(x))
        humid.writeParameter(9, 15000-int(x))
    
    def display_value(self):
        t1=time.time()
        try:
            pt=mc.get_data()
        except:
            print('peltier disconnected')
            peltier=0
        t2=time.time()
        diff=t2-t1
        
        
        while True:
            temperature, humidity = sht4x.single_shot_measurement()
            global t
            global h
            t="{}".format(temperature)
            h="{}".format(humidity)
            if peltier==0:
                pt=None
            else:
                pt=mc.get_data()
            
            if self.radioButton.isChecked()==True:
                #print('checked')
                self.lcdNumber_3.display(float(t[:4]))
                self.lcdNumber_2.display(float(h[:4]))
                if not pt:
                    self.lcdNumber.display(0)
                else:
                    self.lcdNumber.display(float(pt['object temperature'][0]))
                
                time.sleep(.5)
            if self.radioButton.isChecked()==False:
                print('unchecker')
                pt=None
                break
    
    
    def start_display_value(self):
        if self.radioButton.isChecked()==True:
            threading.Thread(target=self.display_value).start()
    
    def set_temp(self):
        mc.set_temp(float(self.spinBox.value()))
    
    def pid(self):
        
        integral=0
        prev_error=0
        error=0
        ku=time.time()
        t1=time.time()
        global f
        f= open(str(self.lineEdit.text())+'.txt', 'w')
        #setpoint=self.doubleSpinBox_4.value()
        setpoint=[10,80,10,80]
        n=0
        while self.pid_flag==True:
            if not posx:
                print('no value detected')
                time.sleep(1)
                ta1=time.time()
                ku=time.time()
            
            else:
                
                x=posx*4.5
                y=posy*4.5
                dist=((x**2)+(y**2))**0.5
                opt_time=time.time()
                ta2=time.time()
                kp=self.doubleSpinBox.value()
                ki=self.doubleSpinBox_2.value()
                kd=self.doubleSpinBox_3.value()
                
                while time.time()-ta2 < self.spinBox_3.value():
                    
                    error=setpoint[n]-(float(h[:4]))
                    print(setpoint[n], error)
                    
                    x=posx*4.5
                    y=posy*4.5
                    dist=((x**2)+(y**2))**0.5
                    
                    error=setpoint[n]-(float(h[:4]))
                    integral=integral+(error*0.2)
                    derivative=error-prev_error
                    prev_error=error
                    output=(error*kp)+(integral*ki)+(derivative*kd)
                    print(output)
                    
                    if output <0:
                        output=0
                        integral=integral-(error*0.2)
                    if output>17000:
                        output=17000
                        integral=integral-(error*0.2)
                    
                    humid.writeParameter(9, int(output))
                    dry.writeParameter(9,17000-int(output))
                    
                    
                    if board.digital[2].read() == 0:
                        light='off'
                    else:
                        light='on'
                    f.write(str(x) + ','+ str(y)+ ',' + str(dist)+ ',' + str(setpoint[n]) + ',' + str(h[:4]) +',' +str(time.time()-ku) + ',' + str((t[:4])))
                    f.write('\n')
                    print('output =' ,output, 'error =' ,error, 'setpoint =', setpoint[n], 'time =', time.time()-ku, 'light =',light)
                    #print('posx =' ,x, 'posy =' ,y, 'setpoint =', setpoint[n], 'time =', time.time()-ku, 'n=', n)
                    time.sleep(0.2)
                ta2=time.time()
                n=n+1
                j=2
                while n>=len(setpoint):
                    
                    while time.time()-ta2 < self.spinBox_3.value():
                        
                        error=setpoint[n-j]-(float(h[:4]))
                        print(setpoint[n-j], error)
                        x=posx*4.5
                        y=posy*4.5
                        dist=((x**2)+(y**2))**0.5
                        error=setpoint[n-j]-(float(h[:4]))
                        integral=integral+(error*0.2)
                        derivative=error-prev_error
                        prev_error=error
                        output=(error*kp)+(integral*ki)+(derivative*kd)
                        print(output)
                        
                        if output <0:
                            output=0
                            integral=integral-(error*0.2)
                        if output>17000:
                            output=17000
                            integral=integral-(error*0.2)
                        
                        humid.writeParameter(9, int(output))
                        dry.writeParameter(9,17000-int(output))
                        
                        
                        if board.digital[2].read() == 0:
                            light='off'
                        else:
                            light='on'
                        f.write(str(x) + ','+ str(y)+ ',' + str(dist)+ ',' + str(setpoint[n-j]) + ',' + str(h[:4]) +',' +str(time.time()-ku) + ',' + str((t[:4])))
                        f.write('\n')
                        print('output =' ,output, 'error =' ,error, 'setpoint =', setpoint[n-j], 'time =', time.time()-ku, 'light =',light)
                        #print('posx =' ,x, 'posy =' ,y, 'setpoint =', setpoint[n-j], 'time =', time.time()-ku, 'n=', n)
                        
                        time.sleep(0.2)
                    ta2=time.time()
                    j=j+1
                    if j>len(setpoint)-1:
                        n=0
    
    def start_pid(self):
        self.pid_flag=True
        threading.Thread(target=self.pid).start()
    
    def stop_pid(self):
        f.close()
        self.pid_flag=False
    
    def opto_stim(self):
        counter=1
        
        while True:
            
            if posx == None:
                start_time=time.time()
                print('opto stim not engaged')
                time.sleep(1)
            
            else:
                opto_timer=time.time()-start_time
                if opto_timer <  ((2*self.spinBox_3.value())+self.spinBox_3.value()/2):
                    counter = counter
                else:
                    counter=counter+1
                    start_time=time.time()
                if counter%2 == 0 :
                    board.digital[2].write(1)
                else:
                    board.digital[2].write(0)
                if self.checkBox_2.isChecked()==False:
                    print('unchecker')
                    board.digital[2].write(0)
                    
                    break
                
    def start_opto_stim(self):
        if self.checkBox_2.isChecked()==True:
            threading.Thread(target=self.opto_stim).start()










if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())