#!/usr/bin/env python3

import rospy
import numpy as np

from apex_controller.locomotion_controller import locomotion_controller

if __name__ == "__main__":
    rospy.init_node("apex_controller_node")
    apex_controller = locomotion_controller()

    start_time = 0
    while not start_time:
        start_time = rospy.Time.now()
    step = False
    apex_controller.stand()
    while not rospy.is_shutdown():
        currentTime = rospy.Time.now()
        deltaT = currentTime - start_time
        #apex_controller.stand()
        #apex_controller.moveFrontRightLeg(0, 180, 10)
        #rospy.sleep(1)
        apex_controller.UpdateMovementSequence(deltaT.to_sec(), 0.1)
        '''if step:
            apex_controller.UpdateMovementSequence(deltaT.to_sec(), 1)
        else:
            apex_controller.UpdateMovementSequence(1-deltaT.to_sec(), 1)
        if(deltaT > rospy.Duration(1)):
            start_time = rospy.Time.now()
            step = not step'''
        '''apex_controller.stand()
        rospy.sleep(4)
        apex_controller.moveFrontRighttLeg(100, 800, 300)
        rospy.sleep(4)'''
