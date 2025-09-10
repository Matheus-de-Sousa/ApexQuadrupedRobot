#!/usr/bin/env python3

import rospy
import time
import board
import adafruit_mpu6050
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_about_axis

IMU_FRAME = None
mpu = None
offsets = None

def calibrate_imu(mpu):
    ax_offset = 0
    ay_offset = 0
    az_offset = 0
    
    gx_offset = 0
    gy_offset = 0
    gz_offset = 0
    
    for i in range(100):
        ax_offset += mpu.acceleration[0]
        ay_offset += mpu.acceleration[1]
        az_offset += mpu.acceleration[2]-9.8
        
        gx_offset += mpu.gyro[0]
        gy_offset += mpu.gyro[1]
        gz_offset += mpu.gyro[2]
        
        time.sleep(0.01)
    
    acc_offsets = [ax_offset,ay_offset,az_offset]
    acc_offsets = [offset/100 for offset in acc_offsets]
    gyro_offsets = [gx_offset, gy_offset, gz_offset]
    gyro_offsets = [offset/100 for offset in gyro_offsets]
    
    return acc_offsets, gyro_offsets

def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME
    
    acc_offsets, gyro_offsets = offsets
    
    imu_msg.linear_acceleration.x = mpu.acceleration[0] - acc_offsets[0]
    imu_msg.linear_acceleration.y = mpu.acceleration[1] - acc_offsets[1]
    imu_msg.linear_acceleration.z = mpu.acceleration[2] - acc_offsets[2]
    
    imu_msg.angular_velocity.x = mpu.gyro[0] - gyro_offsets[0]
    imu_msg.angular_velocity.y = mpu.gyro[1] - gyro_offsets[1]
    imu_msg.angular_velocity.z = mpu.gyro[2] - gyro_offsets[2]
    
    imu_pub.publish(imu_msg)
    
imu_pub = None
if __name__ == '__main__':
    rospy.init_node('imu_node')
    
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    
    print("inicio calibração imu")
    offsets = calibrate_imu(mpu)
    print(offsets[0])
    print(offsets[1])
    print("fim calibração imu")
    
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
    imu_timer = rospy.Timer(rospy.Duration(0.01),publish_imu)
    rospy.spin()
    