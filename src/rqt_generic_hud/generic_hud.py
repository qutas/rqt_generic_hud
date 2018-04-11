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

		for item in self._widget.tabWidget.widget(0).children():
			name = item.objectName()

			if(name == "progress_bar_status"):
				self.progress_bar_status = item
			elif(name == "label_display"):
				self.label_display = item

		for item in self._widget.tabWidget.widget(1).children():
			name = item.objectName()

			if(name == "button_refresh"):
				self.button_refresh = item
			elif(name == "combo_style"):
				pass
			elif(name == "combo_topic_list"):
				self.combo_topic_list = item
			elif(name == "combo_topic_contents"):
				self.combo_topic_contents = item
			elif(name == "textbox_value_max"):
				self.textbox_value_max = item
			elif(name == "textbox_value_min"):
				self.textbox_value_min = item

		self.button_refresh.clicked.connect(self.button_refresh_pressed)
		self.combo_topic_list.currentIndexChanged.connect(self.combo_topic_list_pressed)
		self.combo_topic_contents.currentIndexChanged.connect(self.combo_topic_contents_pressed)

		self.sub = None
		self.msg_in = None

	def shutdown_plugin(self):
		if self.sub is not None:
			self.sub.unregister()

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
	def getKey(self,item):
		return item[0]

	def button_refresh_pressed(self):
		self.msg_in = None

		self.topic_list = sorted(rospy.get_published_topics(), key=self.getKey)

		self.combo_topic_list.clear()
		self.combo_topic_list.addItem("")
		for t in self.topic_list:
			self.combo_topic_list.addItem(t[0])

		self.combo_topic_contents.clear()

	def combo_topic_list_pressed(self):
		self.msg_in = None

		ind = self.combo_topic_list.currentIndex() - 1
		connection_header =  self.topic_list[ind][1].split("/")
		ros_pkg = connection_header[0] + ".msg"
		msg_type = connection_header[1]
		msg_class = getattr(import_module(ros_pkg), msg_type)

		self.sub = rospy.Subscriber(self.topic_list[ind][0], msg_class, self.sub_callback)

		self.combo_topic_contents.clear()
		for s in msg_class.__slots__:
			self.combo_topic_contents.addItem(s)

	def combo_topic_contents_pressed(self):
		self.update_display()

	def update_display(self):
		if self.msg_in is not None:
			try:
				val = float(getattr(self.msg_in, self.combo_topic_contents.currentText()))
				val_min = float(self.textbox_value_min.text());
				val_max = float(self.textbox_value_max.text());
				val_norm = (val - val_min) / (val_max - val_min)
				val_percent = int(100*val_norm)

				self.progress_bar_status.setValue(val_percent)
				self.label_display.setText(str(val_percent))
			except AttributeError:
				pass
			except TypeError:
				pass

	def sub_callback(self, msg_in):
		self.msg_in = msg_in
		self.update_display()


