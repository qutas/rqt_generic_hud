import os
import sys
import math
import rospkg
import rospy

from importlib import import_module

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class GenericHUD(Plugin):
	def __init__(self, context):
		super(GenericHUD, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('GenericHUD')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		#from argparse import ArgumentParser
		#parser = ArgumentParser()
		# Add argument(s) to the parser.
		#parser.add_argument("-q", "--quiet", action="store_true",
		#              dest="quiet",
		#              help="Put plugin in silent mode")
		#args, unknowns = parser.parse_known_args(context.argv())
		#if not args.quiet:
		#    print 'arguments: ', args
		#    print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_generic_hud'), 'resource', 'GenericHUD.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('GenericHUD')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		#self._widget.tabWidget.children()[0].widget(1).layout.button_update.clicked.connect(self.button_update_pressed)
		rospy.loginfo((self._widget.tabWidget.__dict__))
		#self._widget.combo_param_list.currentIndexChanged.connect(self.combo_param_list_pressed)

		self.param_topic_name = ""
		self.param_field_name = ""

		self.reset_subscribers()

	def shutdown_plugin(self):
		if self.binary_sub is not None:
			self.binary_sub.unregister()
		if self.deserialized_sub is not None:
			self.deserialized_sub.unregister()

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def reset_subscribers(self):
		if self.param_topic_name:
			self.binary_sub = rospy.Subscriber(self.param_topic_name, rospy.msg.AnyMsg, self.binary_callback)
		else:
			self.binary_sub = None

		self.deserialized_sub = None

	def button_config_pressed(self):
		rospy.loginfo("DEBUG: Safety arm button pressed!")

	def binary_callback(self, data):
		assert sys.version_info >= (2,7) #import_module's syntax needs 2.7
		connection_header =  data._connection_header["type"].split("/")
		ros_pkg = connection_header[0] + ".msg"
		msg_type = connection_header[1]
		#print 'Message type detected as ' + msg_type
		msg_class = getattr(import_module(ros_pkg), msg_type)
		self.binary_sub.unregister()
		self.deserialized_sub = rospy.Subscriber("~/input", msg_class, self.deserialized_callback)

	def deserialized_callback(self, msg_in):
		try:
			val = int(getattr(msg_in, self.param_field_name))
			self._widget.progress_bar_status.setValue(val)
			self._widget.label_display.setText(str(val) + "%")
		except:
			"Unexpected error:", sys.exc_info()[0]

