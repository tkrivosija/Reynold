#!/usr/bin/env python3
import time
import rospy
from dynamic_reconfigure.server import Server
from reynold.cfg import dynamicVariablesConfig
from std_msgs.msg import Int32

class TestDynamicReconfigure:

    def __init__(self):
        self.dynamicVariables = None  # Initialize the variable as a class attribute
        rospy.init_node('test_dynamicReconfigure')

        srv = Server(dynamicVariablesConfig, self.reconfigure_callback)
        self.rate = rospy.Rate(1)

    def reconfigure_callback(self, config, level):
        self.dynamicVariables = config
        return config

    def run(self):
        while not rospy.is_shutdown():
            if(self.dynamicVariables is not None):
                rospy.loginfo(self.dynamicVariables)
            
            time.sleep(0.1)

if __name__ == '__main__':
    node = TestDynamicReconfigure()
    node.run()

