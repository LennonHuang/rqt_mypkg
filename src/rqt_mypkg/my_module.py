import os
import rospy
import rospkg
import rosbag

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

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

        #adding connection of signal and slot here
        self._widget.btn.pressed.connect(self._test_slot)
        self._widget.btn_stop.pressed.connect(self._test_stop_slot)
        self._widget.bag_btn.pressed.connect(self._test_rosbag)
        self._widget.stop_record_btn.pressed.connect(self._test_stop_rosbag)

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

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
