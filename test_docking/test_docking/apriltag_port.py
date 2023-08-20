import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import transforms3d as tf
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations as tr
import numpy as np
import json
from geometry_msgs.msg import Quaternion

import os
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from geometry_msgs.msg import TransformStamped

import math
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import yaml


class PID:
    def __init__(self, Kp=0, Ki=0, Kd=0):
        '''
        '''
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def proportional_control(self, error):
        return self.Kp * error

    def integral_control(self, error, dt):
        return self.Ki * error * dt

    def derivative_control(self, error, previous_error, dt):
        return self.Kd * (error - previous_error) / dt
        

class Docking(Node):
    def __init__(self):
        super().__init__('docking')
        self.tf_buffer = Buffer()
        self.tf_listenser = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)
        self.spin = True
        self.is_detect = False
        self.move = False
        self.vel = Twist()
        self.saved_time = self.get_clock().now()
        self.prev_error = 0
        kp = 0.05
        ki = 0.1
        kd = 0.3
        self.controller = PID(kp, kd, ki)

        self.translation = {}

    def get_transformation_from_aptag_to_port(self):
        frame = "charger"
        source_frame = "port"
        try:
            transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                            timeout=rclpy.duration.Duration(seconds=2.0))

            translation_values = {
                 "translation_x":transformation.transform.translation.x,
                 "translation_y":transformation.transform.translation.y,
                 "translation_z":transformation.transform.translation.z,
            }
            self.translation.update(translation_values)
            print(self.translation)

        except (LookupException, ConnectivityException, ExtrapolationException):
                pass

    def velocity_control(self, error, dt, prev_error):
        max_vel = 1
        mv_p = self.controller.proportional_control(error)
        mv_i = self.controller.integral_control(error, dt)
        mv_d = self.controller.derivative_control(error, prev_error, dt)

        desired_vel = np.clip(mv_p + mv_i + mv_d, -max_vel, max_vel)
        return desired_vel

    def spin(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.2

    def apriltag_callback(self, msg):
        
        if msg.detections:           
            for at in msg.detections:
                if(at.id ==203):
                    self.is_detect = True 
                    print("True!")            
        else:
            # pass
            self.is_detect = False
            print('No aptags from callback')


    def move_towards_tag(self):
        if self.is_detect is True:
            current_error = float(self.translation.get("translation_y", 0.0))
            transition_x = float(self.translation.get("translation_x", 0.0))
            print("current_error", current_error)
            print("x", transition_x)
            if transition_x>0.05:
                self.vel.linear.x = 0.1
                current_time = self.get_clock().now()
                dt = (current_time - self.saved_time).nanoseconds / 1e9
                pid_output =self.velocity_control(current_error*1.2, dt, self.prev_error)
                print("pid_output ", pid_output)
                self.saved_time = current_time
                if(pid_output>0.3):
                    self.vel.angular.z = 0.3
                if(pid_output<-0.3):
                    self.vel.angular.z = -0.3
                else:
                    self.vel.angular.z = pid_output
                

                self.pub.publish(self.vel)
                self.prev_error = current_error
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z =0.0
                self.pub.publish(self.vel)
        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.2
            self.pub.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    tag_to_bar = Docking()
    # tag_to_bar = Docking()
    # tag_to_bar.get_transformation_from_aptag_to_port()
    # while get_pose.cam_to_base_link is None:
    # get_pose.get_transform_matrix_cam_to_base_link()
    # print('&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&', get_pose.transform_aptag_in_world_dict )
    while( rclpy.ok()):
        tag_to_bar.get_transformation_from_aptag_to_port()
        tag_to_bar.move_towards_tag()
        print("I am calling")
        rclpy.spin_once(tag_to_bar)
        
    
    tag_to_bar.destroy_node()
    rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     tag_to_bar = Docking()

#     executor = MultiThreadedExecutor()
#     tag_to_bar.get_transformation_from_aptag_to_port()
#     tag_to_bar.move_towards_tag()
#     executor.add_node(tag_to_bar)

#     try:
#         executor.spin()
#     finally:
#         executor.shutdown()

#     rclpy.shutdown()

if __name__ == '__main__':
    main()
