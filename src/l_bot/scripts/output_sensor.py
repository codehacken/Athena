#!/usr/bin/env python
"""
This is the ROS Node which acts the Robots output.
AUTHOR: Ashwinkumar Ganesan.
"""

from lib.node import RobotNode
from lib.transport import Message

class OutputNode(RobotNode):
    _input_node_id = 1
    _cpu_id = 2

    def __init__(self, ID, node_name, topic_name):
        super(OutputNode, self).__init__(ID, node_name, topic_name)

    def recv_learn_example(self, message):
        print(message.message)

def listener():
    # Initiate the listener.
    out = OutputNode(3, 'robot_output', '/robot/messages')
    out.start()

if __name__ == '__main__':
    listener()
