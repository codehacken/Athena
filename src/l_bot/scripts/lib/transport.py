#!/usr/bin/env python
"""
Create the transport layer for sending and receiving messages.
Contains the structure of the message and how the messages are processed.
AUTHOR: Ashwinkumar Ganesan.
"""
__author__ = 'Ashwinkumar Ganesan'
import rospy
from l_bot.msg import Control

"""
The class Message provides the structure for the message.
It contains of the following parameters:
1. Source - Int32
2. Destination - Int32
3. Command - String
4. Type - Int32
5. Message - String
"""
class Message(object):
    def __init__(self, source, target, cmd, type, message):
        # Create the message object.
        self.msg = Control()

        # Create the message.
        self.msg.source = source
        self.msg.target = target
        self.msg.command = cmd
        self.msg.type = type
        self.msg.message = message

    def get_message(self):
        return self.msg

"""
Define the transport layer which processes the messages.
The transport layer consists of a single publisher and subscriber.

This is to reduce synchronization issues. The message contains all
the information necessary to work with the messages.

Class Tlayer() requires the following parameters:
1. ID: The ID of the node creating the layer.
2. node_name: The name of the ROS Node started.
3. topic_name: The ROS Topic which publishes and subscribes the data.
   NOTE: The topic name should be consistent across all TLayers.
"""
class TLayer(object):
    # Define the queue size for the publisher.
    _queue_size = 10
    _broadcast = -1

    def __init__(self, ID, topic_name, message_handler):
        self._id = ID
        self.topic = topic_name
        self._message_pub = rospy.Publisher(self.topic, Control,
                                            queue_size=TLayer._queue_size)

        self._message_sub = rospy.Subscriber(self.topic, Control,
                                             self._process_message)

        # Message Handler is the callback function.
        self._m_handle = message_handler

    # Hidden Functions.
    # Process incoming messages to the subscriber.
    def _process_message(self, message):
        # If the source of the message and the target are the same,
        # then reject the message.
        if message.source != self._id:
            # Check if the message is intended or the current Node or broadcast.
            if message.target == self._id or message.target == TLayer._broadcast:
                self._m_handle(message)

    # Publish a message.
    # message should be of type lib.transport.Message
    def send_message(self, message):
        self._message_pub.publish(message.get_message())
