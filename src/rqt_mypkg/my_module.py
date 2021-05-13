import os
import rospy
import rospkg
import rosbag
import serial


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from PyQt5 import QtSerialPort
from PyQt5.QtSerialPort import QSerialPortInfo


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
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() >= 0:
            self._widget.setWindowTitle("Aqua" + self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._bag = None
        #add publisher
        self._publisher = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self._subscriber = None
        self._serial_ports = 0
        self._ser = None

        #adding connection of signal and slot here
        self._widget.btn.pressed.connect(self._test_slot)
        self._widget.btn_stop.pressed.connect(self._test_stop_slot)
        self._widget.bag_btn.pressed.connect(self._test_rosbag)
        self._widget.stop_record_btn.pressed.connect(self._test_stop_rosbag)
        self._widget.serial_scan_btn.pressed.connect(self._serial_scan_btn)
        self._widget.gyro_start_btn.pressed.connect(self._slot_update_gyro)

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
    def _slot_update_gyro(self):
        self._ser = serial.Serial('/dev/ttyACM0',9600)
        self._ser.write("Send\n")
        gyro_data = self._ser.read_until('\n')
        gyro_text = gyro_data.split(";")
        self._widget.x_label.setText(gyro_text[0])
        self._widget.y_label.setText(gyro_text[1])
        self._widget.z_label.setText(gyro_text[2])

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
