import os
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import SIGNAL
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

        # Press buttons.
        self.aButton.pressed.connect(lambda: self.pressButton('A'))
        self.bButton.pressed.connect(lambda: self.pressButton('B'))
        self.xButton.pressed.connect(lambda: self.pressButton('X'))
        self.yButton.pressed.connect(lambda: self.pressButton('Y'))

        self.startButton.pressed.connect(lambda: self.pressButton('START'))
        self.backButton.pressed.connect(lambda: self.pressButton('BACK'))

        self.rbButton.pressed.connect(lambda: self.pressButton('RB'))
        self.lbButton.pressed.connect(lambda: self.pressButton('LB'))

        self.rjButton.pressed.connect(lambda: self.pressButton('RJ'))
        self.ljButton.pressed.connect(lambda: self.pressButton('LJ'))

        # Release buttons.
        self.aButton.released.connect(lambda: self.releaseButton('A'))
        self.bButton.released.connect(lambda: self.releaseButton('B'))
        self.xButton.released.connect(lambda: self.releaseButton('X'))
        self.yButton.released.connect(lambda: self.releaseButton('Y'))

        self.startButton.released.connect(lambda: self.releaseButton('START'))
        self.backButton.released.connect(lambda: self.releaseButton('BACK'))

        self.rbButton.released.connect(lambda: self.releaseButton('RB'))
        self.lbButton.released.connect(lambda: self.releaseButton('LB'))

        self.rjButton.released.connect(lambda: self.releaseButton('RJ'))
        self.ljButton.released.connect(lambda: self.releaseButton('LJ'))

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
        self.ns = ""
        self.updatePublisher()

        self.connect(self.namespaceLineEdit, SIGNAL("editingFinished()"),
                     self.handleNamespaceChanged)

    def pressButton(self, button):
        self.joy_msg.buttons[self.getButtonIdx[button]] = 1
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_pub.publish(self.joy_msg)
        time.sleep(0.1)
        # Release Button again.
#         self.joy_msg.buttons[self.getButtonIdx[button]] = 0
#         self.joy_msg.header.stamp = rospy.Time.now()
#         self.joy_pub.publish(self.joy_msg)

    def releaseButton(self, button):
        self.joy_msg.buttons[self.getButtonIdx[button]] = 0
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_pub.publish(self.joy_msg)

    def handleNamespaceChanged(self):
        self.ns = str(self.namespaceLineEdit.text())
        if not len(self.ns) == 0 and not self.ns[0] == "/":
            self.ns = "/" + self.ns
        self.updatePublisher()

    def updatePublisher(self):
        self.joy_pub = rospy.Publisher(self.ns + "/joy", Joy, queue_size = 100)
