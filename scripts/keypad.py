#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import sys
from select import select
import termios
import tty


class keypadPublisher:
    def __init__(self):
        self.key_pub = rospy.Publisher('keypad', String, queue_size = 1)
        self.settings = self.saveTerminalSettings()

        rospy.loginfo("Keypad Node Started.")
        self.start()


    def getKey(self, settings, timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)


    def restoreTerminalSettings(self, old_settings):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    
    def start(self):
        while not rospy.is_shutdown():
            key = self.getKey(self.settings, 0.5)
        
            if key:
                if key == '\x03':
                    break
                else:
                    key_msg = String()
                    key_msg.data = key
                    self.key_pub.publish(key_msg)
            else:
                continue

        self.restoreTerminalSettings(self.settings)


if __name__ == '__main__':
    try:
        rospy.init_node('keypad', anonymous=True)
        keypad_publisher = keypadPublisher()
    except rospy.ROSInterruptException:
        pass