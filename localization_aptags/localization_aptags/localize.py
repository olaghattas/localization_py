import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from shr_msgs.action import LocalizeRequest
from nav2_msgs.msg import ParticleCloud

import numpy as np
import math
import time
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
import tf_transformations as tr


class LocalizationActionServer(Node):

    def __init__(self):
        super().__init__('Localization_action_server')

        # For localization

        self.vel_pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 10)


        # add the apriltag ids that you used
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.timer_period= 2.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.localize)

        self.publisher_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "ola", 10)
        self.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]


    def publish_pose(self, translation, quat_):
        robot_pose = PoseWithCovarianceStamped()
        quat = Quaternion()
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        robot_pose.header.stamp = self.get_clock().now().to_msg()

        robot_pose.header.frame_id = "map"  # or frame_link

        robot_pose.pose.pose.position.x = translation[0]
        robot_pose.pose.pose.position.y = translation[1]
        robot_pose.pose.pose.position.z = translation[2]

        robot_pose.pose.pose.orientation = quat
        robot_pose.pose.covariance = self.covariance

        self.publisher_initial_pose.publish(robot_pose)

    # def publish_pose_inv(self, translation, quat_):
    #     robot_pose = PoseWithCovarianceStamped()
    #     quat = Quaternion()
    #     quat.x = quat_[0]
    #     quat.y = quat_[1]
    #     quat.z = quat_[2]
    #     quat.w = quat_[3]
    #     robot_pose.header.stamp = self.get_clock().now().to_msg()
    #     robot_pose.header.frame_id = "map"  # or frame_link
    #
    #     robot_pose.pose.pose.position.x = translation[0]
    #     robot_pose.pose.pose.position.y = translation[1]
    #     robot_pose.pose.pose.position.z = translation[1]
    #
    #     robot_pose.pose.pose.orientation = quat.inverse
    #     robot_pose.pose.covariance = self.covariance
    #
    #     self.publisher_initial_pose.publish(robot_pose)

    ##### Localization Part #####
    def publish_tf(self, translation, quat_, child_frame_id, frame_id):
        quat = Quaternion()
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = frame_id
        static_transform.child_frame_id = child_frame_id

        static_transform.transform.translation.x = translation[0]  # Set translation values
        static_transform.transform.translation.y = translation[1]
        static_transform.transform.translation.z = translation[2] # meters

        static_transform.transform.rotation = quat
        self.tf_broadcaster.sendTransform(static_transform)
        # print(f'publishing child {child_frame_id} from {frame_id}')

    def localize(self):
        quat = [0.0,0.0, 0.0, 1.0]
        translation = [-1.1,2.1,0.0]
        self.publish_pose(translation, quat)
        self.publish_tf(translation, quat, "base_link_","map")


def main(args=None):
    rclpy.init(args=args)

    loc_action_server = LocalizationActionServer()

    exe = rclpy.executors.MultiThreadedExecutor()
    exe.add_node(loc_action_server)
    while True:
        exe.spin_once(timeout_sec=0.0)

    loc_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
