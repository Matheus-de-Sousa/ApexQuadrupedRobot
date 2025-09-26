#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

from apex_controller.locomotion_controller import locomotion_controller
from apex_controller.Robot import Apex

def handle_imu_pose(msg):
    global roll, pitch, yaw
    imu_quaternion = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(imu_quaternion)
    roll = (roll/math.pi)*180
    pitch = (pitch/math.pi)*180
    yaw = (yaw/math.pi)*180
    
if __name__ == "__main__":
    rospy.init_node("apex_controller_node")
    rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
    ApexRobot = Apex() 
    apex_controller = locomotion_controller(ApexRobot)

    start_time = 0
    while not start_time:
        start_time = rospy.Time.now()
    step = False
    #apex_controller.stand()
    while not rospy.is_shutdown():
        currentTime = rospy.Time.now()
        deltaT = currentTime - start_time
        #print((roll, pitch, yaw))
        #rospy.sleep(0.01)
        #apex_controller.land()
        #apex_controller.moveFrontRightLeg(0, 180, 10)
        #rospy.sleep(1)
        #apex_controller.SetSingleGait([-90,190,60]) # gait estático
        #apex_controller.SetSingleGait([-60,150,60])
        apex_controller.TrotGaitMovement(deltaT.to_sec(), 0.07)
        #apex_controller.UpdateMovementSequence(deltaT.to_sec(), 0.06)
        #apex_controller.gaitGraph()
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
