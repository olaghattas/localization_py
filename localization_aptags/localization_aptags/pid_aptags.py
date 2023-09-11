import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations as tr
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math


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


class GetPose(Node):

    def __init__(self):
        super().__init__('get_pose')
        self.pub = self.create_publisher(Twist, "cmd_vel", 1)

        self.move = False
        self.vel = Twist()
        self.prev_error = 0
        self.saved_time = self.get_clock().now()
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)
        # self.subscription  # to prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.t_opcam_in_cam = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        kp = 0.05
        ki = 0.1
        kd = 0.3
        self.controller = PID(kp, kd, ki)

        self.ap = None
        self.detected = False
        self.dist_to_tag = 0

    def update_status(self):
        # print("dist_to_tag", self.dist_to_tag)
        if not self.detected:
            self.move = False
            self.spin = True
        elif self.detected and self.dist_to_tag < 0.3:  # limit set based on trials
            self.move = False
            self.spin = True
        else:
            self.spin = False
            self.move = True

    def get_dist(self):
        dist = np.linalg.norm([self.ap[0], self.ap[1], self.ap[2]])
        # dist2 = np.linalg.norm([self.ap_in_baselink[0,3], self.ap_in_baselink[1,3]])
        print('dist', dist)
        return dist

    def velocity_control(self, error, dt, prev_error):
        max_vel = 1
        mv_p = self.controller.proportional_control(error)
        mv_i = self.controller.integral_control(error, dt)
        mv_d = self.controller.derivative_control(error, prev_error, dt)

        desired_vel = np.clip(mv_p + mv_i + mv_d, -max_vel, max_vel)
        return desired_vel

    def spinning(self):
        # adjust the velocity message
        self.vel.angular.z = 0.2
        self.vel.linear.x = 0.0
        # publish it
        self.pub.publish(self.vel)

    def apriltag_callback(self, msg):
        if msg.detections:
            source_frame = msg.header.frame_id  # to
            print("source_frame", source_frame)
            for at in msg.detections:

                frame = "tag_" + str(at.id)  # from
                print(frame)
                try:
                    transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time())
                    # Given translation vector
                    translation_vector = np.array([
                        transformation.transform.translation.x,
                        transformation.transform.translation.y,
                        transformation.transform.translation.z,
                    ])
                    # Convert translation vector to 4x1 column vector
                    translation_vector_4x1 = np.append(translation_vector, 1.0)
                    # Apply transformation
                    transformed_vector = np.dot(self.t_opcam_in_cam, translation_vector_4x1)

                    # Extract the transformed values
                    self.ap = [transformed_vector[0], transformed_vector[1], transformed_vector[2]]
                    self.detected = True
                    self.dist_to_tag = self.get_dist()
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {frame} to {source_frame}: {ex}')
                    return
        else:
            self.ap = None
            self.detected = False

    def move_towards_tag(self):
        current_error = self.ap[1]
        print("current_error", current_error)
        if current_error is not None:
            self.vel.linear.x = 0.25
            current_time = self.get_clock().now()
            dt = (current_time - self.saved_time).nanoseconds / 1e9
            pid_output = self.velocity_control(current_error, dt, self.prev_error)
            # print("pid_output ", pid_output)
            self.vel.angular.z = pid_output
            self.saved_time = current_time

            self.pub.publish(self.vel)
            self.prev_error = current_error


def main(args=None):
    rclpy.init(args=args)
    get_pose = GetPose()

    get_pose.create_rate(2)

    try:
        while rclpy.ok():
            get_pose.update_status()
            if get_pose.spin:
                get_pose.spinning()
            elif get_pose.move:
                get_pose.move_towards_tag()
            rclpy.spin_once(get_pose)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
