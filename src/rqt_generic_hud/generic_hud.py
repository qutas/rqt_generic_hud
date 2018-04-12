import os
import sys
import math
import rospkg
import rospy

from importlib import import_module

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from rqt_generic_hud.generic_hud_options import SimpleSettingsDialog

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

		#for item in self._widget.tabWidget.widget(0).children():
		#	name = item.objectName()

		#	if(name == "progress_bar_status"):
		#		self.progress_bar_status = item
		#	elif(name == "label_display"):
		#		self.label_display = item

		#for item in self._widget.tabWidget.widget(1).children():
		#	name = item.objectName()
		#
		#	if(name == "button_refresh"):
		#		self.button_refresh = item
		#	elif(name == "combo_style"):
		#		pass
		#	elif(name == "combo_topic_list"):
		#		self.combo_topic_list = item
		#	elif(name == "combo_topic_contents"):
		#		self.combo_topic_contents = item
		#	elif(name == "textbox_value_max"):
		#		self.textbox_value_max = item
		#	elif(name == "textbox_value_min"):
		#		self.textbox_value_min = item

		#self.button_refresh.clicked.connect(self.button_refresh_pressed)
		#self.combo_topic_list.currentIndexChanged.connect(self.combo_topic_list_pressed)
		#self.combo_topic_contents.currentIndexChanged.connect(self.combo_topic_contents_pressed)

		self.sub = None

		self.topic_name = ""
		self.topic_type = ""
		self.topic_content = ""
		self.val_min = 0.0
		self.val_max = 1.0

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

	def trigger_configuration(self):
		self.open_settings_dialog()

	def getKey(self,item):
		return item[0]

	def get_topic_type(self, name):
		topics = sorted(rospy.get_published_topics(), key=self.getKey)

		topic_names, topic_types = zip(*topics)
		ind = topic_names.index(name)

		connection_header = topic_types[ind].split("/")
		ros_pkg = connection_header[0] + ".msg"
		msg_type = connection_header[1]
		msg_class = getattr(import_module(ros_pkg), msg_type)

		return msg_class

	def sub_callback(self, msg_in):
		try:
			val = float(getattr(msg_in, self.topic_content))
			val_norm = (val - self.val_min) / (self.val_max - self.val_min)
			val_percent = int(100*val_norm)

			self._widget.progress_bar_status.setValue(val_percent)
			self._widget.label_display.setText(str(val_percent))
		except AttributeError:
			pass
		except TypeError:
			pass

	def open_settings_dialog(self):
		"""Present the user with a dialog for choosing the topic to view,
		the data type, and other settings used to generate the HUD.
		This displays a SimpleSettingsDialog asking the user to choose
		the settings as desired.

		This method is blocking"""

		dialog = SimpleSettingsDialog(title='HUD Options')
		dialog.add_topic_list("topic_list", "Topics")
		dialog.add_combobox_empty("content_list", "Contents")
		dialog.add_lineedit("val_min", "0.0", "Minimum")
		dialog.add_lineedit("val_max", "1.0", "Maximum")

		settings = dialog.get_settings();
		if settings is not None:
			for s in settings:
				if s[0] == "topic_list":
					self.topic_name = str(s[1])
				elif s[0] == "content_list":
					self.topic_content = str(s[1])
				elif s[0] == "val_min":
					self.val_min = float(s[1])
				elif s[0] == "val_max":
					self.val_max = float(s[1])

			if self.topic_name and self.topic_content:
				self.topic_type = self.get_topic_type(self.topic_name)
				self.sub = rospy.Subscriber(self.topic_name, self.topic_type, self.sub_callback)

