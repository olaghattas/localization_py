import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf_transformations as tr
import numpy as np
import json
from pyquaternion import Quaternion
import os
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from geometry_msgs.msg import TransformStamped

import math
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import yaml


class GetPose(Node):

    def __init__(self):
        super().__init__('get_pose')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, "robot_pose", 10)
        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)
        # self.subscription  # to prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.used_apriltags = [0, 2, 13]  # add the apriltag ids that you used
        self.transform_aptag_in_cam_dict = {}  # location of apriltags in camera frame
        self.transform_aptag_in_world_dict = {}  # global location of apriltags

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.closest_aptag = None
        self.cam_to_base_link = None
        # incase we decided to get this in another way
        self.t_cam_in_baselink = None
        self.t_opcam_in_cam = None

    def yaw_to_quaternion(self, yaw):
        half_yaw = yaw * 0.5
        cos_half_yaw = np.cos(half_yaw)
        sin_half_yaw = np.sin(half_yaw)

        quaternion = np.array([cos_half_yaw, 0, 0, sin_half_yaw])
        return quaternion

    def publish_tf(self, robot_pose_aptags):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'map'

        static_transform.transform.translation.x = robot_pose_aptags[0]  # Set translation values
        static_transform.transform.translation.y = robot_pose_aptags[1]
        static_transform.transform.translation.z = 0.5  # meters
        quat = self.yaw_to_quaternion(robot_pose_aptags[2])
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(static_transform)

    def dist(self, x, y, z):
        return np.linalg.norm([x, y, z])

    def get_transform_matrix_aptags_from_tf(self):
        for aptag in self.used_apriltags:
            str_aptag = str(aptag)
            source_frame = "map"  # to
            frame = "aptag_" + str_aptag  # from

            try:
                transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time())
                print('jsk', transformation)

                translation = tr.translation_matrix(
                    [transformation.transform.translation.x, transformation.transform.translation.y,
                     transformation.transform.translation.z])
                rotation = tr.quaternion_matrix(
                    [transformation.transform.rotation.x, transformation.transform.rotation.y,
                     transformation.transform.rotation.z, transformation.transform.rotation.w])
                # Get the homogeneous transformation matrix
                transform_aptag_in_world = np.dot(translation, rotation)
                self.transform_aptag_in_world_dict[str_aptag] = transform_aptag_in_world

            except (LookupException, ConnectivityException, ExtrapolationException):
                self.get_logger().info('transform not ready')
                self.get_logger().info(
                    f'Could not transform {frame} to {source_frame}')
                # raise
                return

            print('self.transform_aptag_in_cam_dict', self.transform_aptag_in_cam_dict)

    def get_transform_matrix_cam_to_base_link(self):
        source_frame = "map"  # to
        frame = "base_link" # from

        try:
            transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time())

            translation = tr.translation_matrix(
                [transformation.transform.translation.x, transformation.transform.translation.y,
                 transformation.transform.translation.z])
            rotation = tr.quaternion_matrix(
                [transformation.transform.rotation.x, transformation.transform.rotation.y,
                 transformation.transform.rotation.z, transformation.transform.rotation.w])
            # Get the homogeneous transformation matrix
            transform_cam_to_base_link = np.dot(translation, rotation)
            self.cam_to_base_link = transform_cam_to_base_link

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            self.get_logger().info(
                f'Could not transform {frame} to {source_frame}')
            # raise
            return

    def apriltag_callback(self, msg):
        min_distance = np.inf
        if msg.detections:
            self.transform_aptag_in_cam_dict = {}
            source_frame = msg.header.frame_id  # to
            for at in msg.detections:

                frame = "tag_" + str(at.id)  # from
                print(frame)
                try:
                    transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time())
                    print('jsk', transformation)

                    dist = dist([transformation.transform.translation.x, transformation.transform.translation.y,
                                           transformation.transform.translation.z])
                    if dist < min_distance:
                        min_distance = dist
                        self.closest_aptag = at.id

                    translation = tr.translation_matrix(
                        [transformation.transform.translation.x, transformation.transform.translation.y,
                         transformation.transform.translation.z])
                    rotation = tr.quaternion_matrix(
                        [transformation.transform.rotation.x, transformation.transform.rotation.y,
                         transformation.transform.rotation.z, transformation.transform.rotation.w])
                    # Get the homogeneous transformation matrix
                    transform_aptag_in_cam = np.dot(translation, rotation)

                    self.transform_aptag_in_cam_dict[at.id] = transform_aptag_in_cam

                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {frame} to {source_frame}: {ex}')
                    return

            print('self.transform_aptag_in_cam_dict', self.transform_aptag_in_cam_dict)

    def transform_cam_world_frame(self):
        robot_position = []
        transform_aptag_in_cam_dict = self.transform_aptag_in_cam_dict
        for aptag in transform_aptag_in_cam_dict.keys():
            t_apriltag_to_world = self.transform_aptag_in_world_dict[aptag]
            t_apriltag_to_camera = transform_aptag_in_cam_dict[aptag]

            t_cam_in_world = np.dot(t_apriltag_to_world, np.linalg.inv(t_apriltag_to_camera))
            t_robot_in_world = np.dot(t_cam_in_world, self.cam_to_base_link)

            # Extract the robot coordinates and rotation from the transformation matrix
            robot_x = t_robot_in_world[0, 3]
            robot_y = t_robot_in_world[1, 3]

            # Append the column vector to the robot_position array
            robot_position.append([robot_x, robot_y])

            if aptag == self.closest:
                # no average for theta jst take the one of the closest aptag
                rotation_matrix = t_robot_in_world[:3, :3]
                rotation_quaternion = tr.quaternion_from_matrix(rotation_matrix)
                theta = rotation_quaternion[3]

        # Convert the robot_position list to a NumPy array
        robot_position_array = np.array(robot_position)

        # Compute the mean for each row to get average of position computed from different aptags
        mean_values = np.mean(robot_position_array, axis=0)
        mean_values.append(theta)
        print(mean_values)
        return mean_values

    def timer_callback(self):
        # if aprtiltags are detected im the scene
        if self.transform_aptag_in_cam_dict:
            # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
            robot_pose_aptags = self.transform_cam_world_frame()

            robot_pose = PoseWithCovarianceStamped()

            current_time = self.get_clock().now()
            robot_pose.header.stamp.sec = int(current_time)
            robot_pose.header.stamp.nanosec = int((current_time - int(current_time)) * 1e9)
            robot_pose.header.frame_id = "map"  # or frame_link

            robot_pose.pose.pose.position.x = robot_pose_aptags[0]
            robot_pose.pose.pose.position.y = robot_pose_aptags[1]
            robot_pose.pose.pose.orientation.w = robot_pose_aptags[2]

            self.publisher_.publish(robot_pose)
            self.publish_tf(robot_pose_aptags)

        else:
            self.get_logger().info('NO apriltags detected')


def main(args=None):
    rclpy.init(args=args)
    get_pose = GetPose()
    get_pose.get_transform_matrix_cam_to_base_link()
    get_pose.get_transform_matrix_aptags_from_tf()

    rclpy.spin(get_pose)


if __name__ == '__main__':
    main()
