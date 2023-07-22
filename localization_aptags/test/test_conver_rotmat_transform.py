import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
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


class GetPose(Node):

    def __init__(self):
        super().__init__('get_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

    def publish_tf(self, x, y, z, quat_, link):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'
        static_transform.child_frame_id = link

        quat = Quaternion()

        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        static_transform.transform.translation.x = x  # Set translation values
        static_transform.transform.translation.y = y
        static_transform.transform.translation.z = z  # meters
        static_transform.transform.rotation = quat

        self.tf_broadcaster.sendTransform(static_transform)

    def rotation_matrix_to_quaternion(self, R):
        # Convert a 3x3 rotation matrix to a Quaternion
        trace = np.trace(R)
        if trace > 0:
            S = 2.0 * math.sqrt(trace + 1.0)
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        return [qx, qy, qz, qw]

    def transform_matrix_to_quat(self, matrix):

        # Extract the rotation matrix and translation vector
        rotation_matrix = matrix[:3, :3]
        translation_vector = matrix[:3, 3]

        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

        # Create a TransformStamped message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp to the current time
        transform_msg.header.frame_id = 'map'  # Set the frame_id

        # Set the translation
        transform_msg.transform.translation.x = translation_vector[0]
        transform_msg.transform.translation.y = translation_vector[1]
        transform_msg.transform.translation.z = translation_vector[2]

        # Set the quaternion
        transform_msg.transform.rotation.x = quaternion[0]
        transform_msg.transform.rotation.y = quaternion[1]
        transform_msg.transform.rotation.z = quaternion[2]
        transform_msg.transform.rotation.w = quaternion[3]
        print('Translation:', translation_vector[0], translation_vector[1], translation_vector[2])
        print('Rotation:', quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        self.publish_tf(translation_vector[0], translation_vector[1], translation_vector[2], quaternion, 'link1')
        # Extract the robot coordinates and rotation from the transformation matrix
        return transform_msg

    def transform_quat_to_matrix(self, transformation):

        translation = tr.translation_matrix(
            [transformation.transform.translation.x, transformation.transform.translation.y,
             transformation.transform.translation.z])

        normalized_quat = [transformation.transform.rotation.x, transformation.transform.rotation.y,
                           transformation.transform.rotation.z, transformation.transform.rotation.w] / np.linalg.norm(
            [transformation.transform.rotation.x, transformation.transform.rotation.y,
             transformation.transform.rotation.z, transformation.transform.rotation.w])

        rotation = tr.quaternion_matrix(normalized_quat)
        # Get the homogeneous transformation matrix
        transform_aptag_in_cam = np.dot(translation, rotation)
        print('in tf', np.around(transform_aptag_in_cam, decimals=2))

        return transform_aptag_in_cam


def main(args=None):
    rclpy.init(args=args)
    get_pose = GetPose()

    # Sample data_matrix with multiple matrices
    data_matrix = [
        np.array([
            [0.0, 1.0, 0.0, 1.08],
            [-1.0, 0.0, 0.0, 1.65],
            [0.0, 0.0, 1.0, 0.15],
            [0.0, 0.0, 0.0, 1.0]
        ]),

    ]

    for idx, data in enumerate(data_matrix):
        numpy_matrix = np.array(data)
        print('numpy_matrix ' + str(idx), numpy_matrix)

        transform_msg = get_pose.transform_matrix_to_quat(numpy_matrix)

        transform_aptag_in_cam = get_pose.transform_quat_to_matrix(transform_msg)

        # Round the elements to the desired precision (e.g., 2 decimal places)
        rounded_transform_aptag_in_cam = np.around(transform_aptag_in_cam, decimals=2)

        print('transform_aptag_in_cam ' + str(idx), rounded_transform_aptag_in_cam)

    rclpy.spin(get_pose)
    rclpy.shutdown()

    def yaw_to_quaternion(self, yaw):
        half_yaw = yaw * 0.5
        cos_half_yaw = np.cos(half_yaw)
        sin_half_yaw = np.sin(half_yaw)

        quaternion = np.array([cos_half_yaw, 0, 0, sin_half_yaw])
        return quaternion

if __name__ == '__main__':
    main()
