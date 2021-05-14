import os
import rospy
import rospkg
import rosbag
import serial
import rviz

from serial import SerialException
from qt_gui.plugin import Plugin
#from python_qt_binding import loadUi
#from python_qt_binding.QtWidgets import QWidget, QFileDialog
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from PyQt5.QtWidgets import QWidget, QFileDialog
from PyQt5 import uic, QtSerialPort
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtSerialPort import QSerialPortInfo
from nano_worker import nano_worker

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        uic.loadUi(ui_file, self._widget)
        #loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        #self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        #if context.serial_number() >= 0:
            #self._widget.setWindowTitle("Aqua" + self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._bag = None
        #add publisher
        self._publisher = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self._subscriber = None
        self._serial_ports = 0
        self._nanoWorker = None
        self._nanoThread = None
        self._widget.led_btn.setEnabled(False)
        

        #adding connection of signal and slot here
        self._widget.btn.clicked.connect(self._test_slot)
        self._widget.btn_stop.clicked.connect(self._test_stop_slot)
        self._widget.bag_btn.clicked.connect(self._test_rosbag)
        self._widget.stop_record_btn.clicked.connect(self._test_stop_rosbag)
        self._widget.serial_scan_btn.clicked.connect(self._serial_scan_btn)
        self._widget.imu_start_btn.clicked.connect(self._slot_update_imu)
        self._widget.imu_stop_btn.clicked.connect(self._slot_imu_stop)
        self._widget.imu_refresh_btn.clicked.connect(self._slot_imu_refresh)
        self._widget.led_btn.clicked.connect(self._slot_led_update)
        

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #Slots
    def _test_slot(self):
        self._widget.label.setText("Pressed")
        t = Twist()
        t.angular.z = 66
        self._publisher.publish(t)
        print("Someone pressed me")

    def _test_stop_slot(self):
        self._widget.stop_label.setText("Stop!")
        t = Twist()
        t.angular.z = 0
        self._publisher.publish(t)
        print("Stop!")
    def _test_rosbag(self):
        filename = QFileDialog.getSaveFileName(self._widget,self._widget.tr('Select name for the bag'),"",self._widget.tr('Bag files {.bag} (*.bag)'))[0]
        print(filename)
        if not filename.endswith('.bag'):
            filename += ".bag"
        self._bag = rosbag.Bag(filename,'w')
        self._subscriber = rospy.Subscriber("cmd_vel",Twist,self._bag_callback)
        self._widget.record_status_label.setText("Recording")
        
        print("rosbag recording")
    def _test_stop_rosbag(self):
        self._bag.close()
        print("rosbag stop recording")
        self._widget.record_status_label.setText("Stop recording")
    def _bag_callback(self,data):
        self._bag.write("whatever",data)
    def _serial_scan_btn(self):
        info_list = QSerialPortInfo()
        serial_list = info_list.availablePorts()
        self._serial_ports = [port.portName() for port in serial_list]
        #print(self._serial_ports[0])
        self._widget.comboBox.clear()
        self._widget.comboBox.addItems(self._serial_ports)
    def _slot_update_imu(self):
        #initialize the thread and the worker, and move the worker to the thread.
        self._nanoThread = QThread()
        self._nanoWorker = nano_worker()
        self._nanoWorker.moveToThread(self._nanoThread)
        #Connect the thread start() and worker customized run()
        self._nanoThread.started.connect(self._nanoWorker.run)
        self._nanoWorker.finished.connect(self._nanoThread.exit)
        self._nanoWorker.progress.connect(lambda imu_data: self.imu_reportProgress(imu_data))
        self._nanoWorker.gyro_not_connected.connect(self._slot_imu_notFound)
        

        if(not self._nanoWorker.is_connected):
            print("Disable the button")
            self._nanoWorker.gyro_not_connected.emit()
            self._nanoWorker.finished.emit()
            self._widget.led_btn.setEnabled(False)
        else:
            self._nanoThread.start()
            self._widget.imu_status_label.setText("IMU Started")
            self._widget.imu_stop_btn.setEnabled(True)
            self._widget.imu_refresh_btn.setEnabled(False)
            self._widget.led_btn.setEnabled(True)
        
    def imu_reportProgress(self, imu_data):
        imu_text = imu_data.split(";")
        self._widget.x_label.setText("X: " + imu_text[0] + " deg/s")
        self._widget.y_label.setText("Y: " + imu_text[1] + " deg/s")
        self._widget.z_label.setText("Z: " + imu_text[2] + " deg/s")
        self._widget.x_label_acc.setText("X: " + imu_text[3] + " g")
        self._widget.y_label_acc.setText("Y: " + imu_text[4] + " g")
        self._widget.z_label_acc.setText("Z: " + imu_text[5].rstrip("\n") + " g")

    def _slot_imu_stop(self):
        self._nanoWorker.processing = False
        self._nanoWorker.finished.emit()
        if (self._widget.imu_start_btn.isEnabled()):
            self._widget.imu_status_label.setText("IMU Stopped")
        
        print("Stop the gyro Worker")
        self._widget.imu_refresh_btn.setEnabled(True)
        self._widget.led_btn.setEnabled(False)

    def _slot_imu_notFound(self):
        self._widget.imu_start_btn.setEnabled(False)
        self._widget.imu_stop_btn.setEnabled(True)
        self._widget.imu_status_label.setText("IMU Not Found")

    def _slot_imu_refresh(self):
        self._widget.imu_start_btn.setEnabled(True)
        self._widget.imu_stop_btn.setEnabled(False)
        self._widget.imu_status_label.setText("Refreshed")

    def _slot_led_update(self):
        if(not self._nanoWorker.is_led_on):
            self._nanoWorker.is_led_on = True
            self._widget.led_btn.setText("LED OFF")

        else:
            self._nanoWorker.is_led_on = False
            self._widget.led_btn.setText("LED ON")
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
