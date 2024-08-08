#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client
from std_srvs.srv import Empty

import os
import threading
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper
from PIL import Image, ImageDraw, ImageFont


ASSETS_PATH = os.path.join(os.path.dirname(__file__), "Assets")


def render_key_image(deck, icon_filename):
    icon = Image.open(icon_filename)
    image = PILHelper.create_scaled_key_image(deck, icon, margins=[0, 0, 0, 0])
    
    return PILHelper.to_native_key_format(deck, image)


# Key ID: 0  1  2  3  4
#         5  6  7  8  9
#         10 11 12 13 14
def key_change_callback(deck, key, state):
    response = ""

    if state:
        clear_costmap()

        if key == 0:
            response = "home"

            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = 0
            goal_msg.pose.position.y = 0
            goal_msg.pose.position.z = 0
            goal_msg.pose.orientation.x = 0
            goal_msg.pose.orientation.y = 0
            goal_msg.pose.orientation.z = 0
            goal_msg.pose.orientation.w = 1
            goal_pub.publish(goal_msg)

        elif key == 1:
            response = "point_1"
            
            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = 8.5
            goal_msg.pose.position.y = 8.775
            goal_msg.pose.position.z = 0
            goal_msg.pose.orientation.x = 0
            goal_msg.pose.orientation.y = 0
            goal_msg.pose.orientation.z = 0
            goal_msg.pose.orientation.w = 1
            goal_pub.publish(goal_msg)

        elif key == 2:
            response = "point_2"

        elif key == 4:
            response = "charger"

            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = -1.5
            goal_msg.pose.position.y = 17
            goal_msg.pose.position.z = 0
            goal_msg.pose.orientation.x = 0
            goal_msg.pose.orientation.y = 0
            goal_msg.pose.orientation.z = 0
            goal_msg.pose.orientation.w = 1
            goal_pub.publish(goal_msg)

        elif key == 5:
            response = "navigation"
            client.update_configuration({"max_vel_x": 0.8, "max_trans_vel": 0.8, "acc_lim_x": 0.8, "acc_limit_trans": 0.8})

        elif key == 6:
            response = "followme"
            client.update_configuration({"max_vel_x": 1.5, "max_trans_vel": 1.5, "acc_lim_x": 1.5, "acc_limit_trans": 1.5})

    if response:
        response_pub.publish(response)


if __name__ == "__main__":
    rospy.init_node('stream_deck_node')

    response_pub = rospy.Publisher("keypad_response", String, queue_size=1)
    goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

    client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=None, config_callback=None, description_callback=None)

    clear_costmap = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

    streamdecks = DeviceManager().enumerate()

    rospy.loginfo("Found {} Stream Deck(s).\n".format(len(streamdecks)))

    for index, deck in enumerate(streamdecks):
        deck.open()

        rospy.loginfo("Opened '{}' device (serial number: '{}', fw: '{}')".format(
            deck.deck_type(), deck.get_serial_number(), deck.get_firmware_version()
        ))

        deck.set_key_image(0, render_key_image(deck, os.path.join(ASSETS_PATH, "house.png")))
        deck.set_key_image(1, render_key_image(deck, os.path.join(ASSETS_PATH, "one.png")))
        deck.set_key_image(2, render_key_image(deck, os.path.join(ASSETS_PATH, "two.png")))
        deck.set_key_image(4, render_key_image(deck, os.path.join(ASSETS_PATH, "battery.png")))
        deck.set_key_image(5, render_key_image(deck, os.path.join(ASSETS_PATH, "robot.png")))
        deck.set_key_image(6, render_key_image(deck, os.path.join(ASSETS_PATH, "walking.png")))
            
        # Register callback function for when a key state changes.
        deck.set_key_callback(key_change_callback)

    rospy.spin()

    deck.reset()
    deck.close()