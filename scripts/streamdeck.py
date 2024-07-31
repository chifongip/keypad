#!/usr/bin/env python3

import rospy
import threading
from StreamDeck.DeviceManager import DeviceManager
from std_msgs.msg import String


# Key ID: 0  1  2  3  4
#         5  6  7  8  9
#         10 11 12 13 14
def key_change_callback(deck, key, state):
    response = "Deck {} Key {} = {}".format(deck.id(), key, "down" if state else "up")
    rospy.loginfo(response)
    response_pub.publish(response)


if __name__ == "__main__":
    rospy.init_node('stream_deck_node')

    response_pub = rospy.Publisher('keypad_response', String, queue_size=10)

    streamdecks = DeviceManager().enumerate()

    rospy.loginfo("Found {} Stream Deck(s).\n".format(len(streamdecks)))

    for index, deck in enumerate(streamdecks):
        deck.open()

        rospy.loginfo("Opened '{}' device (serial number: '{}', fw: '{}')".format(
            deck.deck_type(), deck.get_serial_number(), deck.get_firmware_version()
        ))

        # Register callback function for when a key state changes.
        deck.set_key_callback(key_change_callback)

    rospy.spin()