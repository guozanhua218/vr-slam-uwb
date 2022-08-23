#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


class EstopManager():
    def __init__(self):
        self.xbee_engage = True
        self.xbee_stop = False
        self.estop_husky = False


        sub_xbee = rospy.Subscriber(
            "e_stop_xbee", Bool, self.cb_xbee, queue_size=1)
        sub_joy = rospy.Subscriber(
            "joy_teleop/joy", Joy, self.cb_joy, queue_size=1)

        self.pub_estop = rospy.Publisher("e_stop", Bool, queue_size=1)

    def estop(self):
        self.pub_estop.publish(self.estop_husky | (self.xbee_engage & self.xbee_stop))

    def cb_xbee(self, msg):
        self.xbee_stop = msg.data

        self.estop()

    def cb_joy(self, msg):
        # MODE X
        start_button = 7
        back_button = 6
        Logitech = 8
        RB = 5
        LB = 4
        A = 0
        B = 1
        X = 2
        Y = 3

        if msg.buttons[B]==1:
            self.xbee_engage = True
            rospy.loginfo("xbee engage")
        elif msg.buttons[Y]==1:
            self.xbee_engage = False       

        if msg.buttons[A] == 1 and msg.buttons[Logitech] == 1:
            self.estop_husky = False
            rospy.loginfo("local estop")
        elif msg.buttons[Logitech] == 1:
            self.estop_husky = True

        self.estop()

if __name__ == "__main__":
    rospy.init_node("estop_manager")
    manager = EstopManager()
    rospy.spin()
