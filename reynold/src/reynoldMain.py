#!/usr/bin/env python3
import time
import rospy
from dynamic_reconfigure.server import Server
from reynold.cfg import dynamicVariablesConfig
from std_msgs.msg import Int32

class ReynoldMain:

    def __init__(self):
        self.dynamicVariables = None  # Initialize the variable as a class attribute
        rospy.init_node('reynoldMain')

        srv = Server(dynamicVariablesConfig, self.reconfigure_callback)
        self.rate = rospy.Rate(1)

    def reconfigure_callback(self, config, level):
        #self.dynamicVariables = config
        #if(self.dynamicVariables.reboot_params is True):
        rospy.loginfo("Bla")
        return config

    def run(self):
        while not rospy.is_shutdown():
            #ukoliko su se učitale dinamičke varijable i dopušten je globalni enabl vrti glavnu petlju...
            if(self.dynamicVariables is not None and self.dynamicVariables.globalEnable):
                rospy.loginfo(self.dynamicVariables.globalEnable)
                rospy.loginfo("Bla")
            
            time.sleep(0.1)
        rospy.spin()

if __name__ == '__main__':
    node = ReynoldMain()
    node.run()

