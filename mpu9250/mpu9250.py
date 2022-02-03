import os
import sys
import time
import smbus
import numpy as np

from .imusensor.MPU9250 import MPU9250
from .imusensor.filters import kalman 
from .imusensor.filters import madgwick

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from math import sin, cos, radians
import tf_transformations


class MyPythonNode(Node):
    def __init__(self):
        super().__init__("my_node_name")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 30),
                ('frame_id', 'imu_link'),
                ('i2c_address', 0x68),
                ('i2c_port', 1),
                ('acceleration_scale', [1.0, 1.0, 1.0]),
                ('acceleration_bias', [0.0, 0.0, 0.0]),
                ('gyro_bias', [0.0, 0.0, 0.0]),
                ('magnetometer_scale', [1.0, 1.0, 1.0]),
                ('magnetometer_bias', [1.0, 1.0, 1.0]),
                ('magnetometer_transform', [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]),
            ]
        )

        address = self.get_parameter('i2c_address')._value
        bus = smbus.SMBus(self.get_parameter('i2c_port')._value)
        self.imu = MPU9250.MPU9250(bus, address)

        self.imu.Accels = np.asarray(self.get_parameter('acceleration_scale')._value)
        self.imu.AccelBias = np.asarray(self.get_parameter('acceleration_bias')._value)
        self.imu.GyroBias = np.asarray(self.get_parameter('gyro_bias')._value)
        self.imu.Mags = np.asarray(self.get_parameter('magnetometer_scale')._value)
        self.imu.MagBias = np.asarray(self.get_parameter('magnetometer_bias')._value)
        self.imu.Magtransform = np.reshape(np.asarray(self.get_parameter('magnetometer_transform')._value),(3,3))

        self.publisher_imu_values_ = self.create_publisher(Imu, "/imu", 10)
        self.timer_publish_imu_values_ = self.create_timer(
            1.0/self.get_parameter('frequency')._value, self.publish_imu_values)

        self.sensorfusion = kalman.Kalman()
        #self.sensorfusion = madgwick.Madgwick(0.5)
        self.imu.begin()
        self.imu.readSensor()    
        self.imu.computeOrientation()
        self.sensorfusion.roll = self.imu.roll
        self.sensorfusion.pitch = self.imu.pitch
        self.sensorfusion.yaw = self.imu.yaw
        self.deltaTime = 0
        self.lastTime = self.get_clock().now()

    def publish_imu_values(self):
        self.imu.readSensor()
        #self.imu.computeOrientation()
        msg = Imu()
        deltaTime = (self.get_clock().now() - self.lastTime).nanoseconds * 10e9
        self.lastTime = self.get_clock().now()
        #yaw = self.imu.yaw
        #pitch = self.imu.pitch
        #roll = self.imu.roll
        #computeAndUpdateRollPitchYaw
        self.sensorfusion.computeAndUpdateRollPitchYaw(\
            self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],\
            self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],\
	        self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], deltaTime)
        yaw = self.sensorfusion.yaw
        pitch = self.sensorfusion.pitch
        roll = self.sensorfusion.roll

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id')._value
        # Direct measurements
        msg.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        msg.linear_acceleration.x = self.imu.AccelVals[0]
        msg.linear_acceleration.y = self.imu.AccelVals[1]
        msg.linear_acceleration.z = self.imu.AccelVals[2]
        msg.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        msg.angular_velocity.x = (self.imu.GyroVals[0]) #TODO this is acceleration not velocity (?!)
        msg.angular_velocity.y = (self.imu.GyroVals[1]) #TODO this is acceleration not velocity (?!)
        msg.angular_velocity.z = (self.imu.GyroVals[2]) #TODO this is acceleration not velocity (?!)
        #msg.angular_velocity.x = 0.0 #TODO this is acceleration not velocity (?!)
        #msg.angular_velocity.y = 0.0 #TODO this is acceleration not velocity (?!)
        #msg.angular_velocity.z = 0.0 #TODO this is acceleration not velocity (?!)
        # Calculate euler angles, convert to quaternion and store in message
        msg.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        # Convert to quaternion
        quat = tf_transformations.quaternion_from_euler(
            radians(roll), radians(pitch), radians(yaw))
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        self.publisher_imu_values_.publish(msg)
        print("roll: {:4.2f} \tpitch : {:4.2f} \tyaw : {:4.2f}".format(self.sensorfusion.roll, self.sensorfusion.pitch, self.sensorfusion.yaw))

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()