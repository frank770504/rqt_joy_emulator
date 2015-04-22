import os
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QGraphicsView, QWidget

from sensor_msgs.msg import Joy

class MyGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(MyGraphicsView, self).__init__()

class MyWidget(QWidget):

    def __init__(self):
        # set up qt related things
        super(MyWidget, self).__init__()
        self.setObjectName('Joy Emulator')

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_joy_emulator'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self, {'MyGraphicsView': MyGraphicsView})

        self.setWindowTitle('Joy Emulator')

        image_path = os.path.join(rospkg.RosPack().get_path('rqt_joy_emulator'), 'resource', 'f310-gaming-gamepad-images.png')
        style_sheet = ".QWidget { background: url(\"%s\"); }" % image_path
        self.widget.setStyleSheet(style_sheet)

        self.aButton.clicked[bool].connect(self._handle_a_clicked)
        self.bButton.clicked[bool].connect(self._handle_b_clicked)
        self.xButton.clicked[bool].connect(self._handle_x_clicked)
        self.yButton.clicked[bool].connect(self._handle_y_clicked)

        self.startButton.clicked[bool].connect(self._handle_start_clicked)
        self.backButton.clicked[bool].connect(self._handle_back_clicked)

        self.rbButton.clicked[bool].connect(self._handle_rb_clicked)
        self.lbButton.clicked[bool].connect(self._handle_lb_clicked)

        self.rjButton.clicked[bool].connect(self._handle_rj_clicked)
        self.ljButton.clicked[bool].connect(self._handle_lj_clicked)

        # Logitech Wired
        self.getButtonIdx = {'A':0,
                             'B':1,
                             'X':2,
                             'Y':3,
                             'START':7,
                             'BACK':6,
                             'RB':5,
                             'LB':4,
                             'RJ':10,
                             'LJ':9}

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size = 100)

    def pressButton(self, button):
        self.joy_msg.buttons[self.getButtonIdx[button]] = 1
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_pub.publish(self.joy_msg)
        time.sleep(0.1)
        # Release Button again.
        self.joy_msg.buttons[self.getButtonIdx[button]] = 0
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_pub.publish(self.joy_msg)

    def _handle_a_clicked(self):
        self.pressButton('A')

    def _handle_b_clicked(self):
        self.pressButton('B')

    def _handle_x_clicked(self):
        self.pressButton('X')

    def _handle_y_clicked(self):
        self.pressButton('Y')

    def _handle_start_clicked(self):
        self.pressButton('START')

    def _handle_back_clicked(self):
        self.pressButton('BACK')

    def _handle_rb_clicked(self):
        self.pressButton('RB')

    def _handle_lb_clicked(self):
        self.pressButton('LB')

    def _handle_rj_clicked(self):
        self.pressButton('RJ')

    def _handle_lj_clicked(self):
        self.pressButton('LJ')
