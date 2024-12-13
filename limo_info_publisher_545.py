#!/usr/bin/env python3 #vim code and :ff=unix then :wq
import rospy
from pylimo import limo
import time
import numpy as np
from cav_project.msg import limo_info, limo_info_array, ControlInfo


class PubSub:
    def __init__(self, ID):
        self.ID = ID
        rospy.init_node('test')
        self.control_info_topic = rospy.Publisher("control_info_"+self.ID, ControlInfo, queue_size=10)
        self.rate = rospy.Rate(10)

    def publish_info(self):
        self.info_pub_cav1.publish(self.limo_data_cav1)


def msg_format(): #add input from Carla
    msgCI = ControlInfo()
    #CARLA data extrapolation here!
    msgCI.steering_angle = 0.5 #eventually from Carla
    msgCI.desired_velocity = 1 #eventually from Carla
    msgCI.control_input = 1 #eventually from Carla
    return msgCI

if __name__ == '__main__':
    pubsub = PubSub("limo770")
    while not rospy.is_shutdown():
        msgCI = msg_format()
        pubsub.contro_info_topic.publish(msgCI)
        time.sleep(0.1)
    print("publishing ended")