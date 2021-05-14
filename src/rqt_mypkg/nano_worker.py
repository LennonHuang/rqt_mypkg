import rospy
import rospkg
import rosbag
import serial

from serial import SerialException
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from PyQt5 import QtSerialPort
from PyQt5.QtCore import QObject, QThread, pyqtSignal

class nano_worker(QObject):
    #nano worker will emit the progress signal to main thread to trigger slot on MainWindow
    progress = pyqtSignal(str)
    finished = pyqtSignal()
    gyro_not_connected = pyqtSignal()
    is_connected = True
    processing = True
    is_led_on = False

    def __init__(self):
        super(nano_worker, self).__init__()
        try:
            self._ser = serial.Serial('/dev/ttyACM0',9600)
        except SerialException:
            self.processing = False
            self.is_connected = False
            print("Arduino not connected.")

    #Run in a loop
    def run(self):
        while self.processing and self._ser.is_open:
            if(self.is_led_on):
                self._ser.write("11\n")
            else:
                self._ser.write("01\n")
            imu_data = self._ser.read_until('\n')
            self.progress.emit(imu_data)
            print(imu_data)
        self.finished.emit()