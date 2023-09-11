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
        # quaternion PoseWithCovarianceStamped
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)
        self.subscription  # to prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # self.used_apriltags = [0, 1, 2, 3, 4, 5, 6, 7, 8]  # add the apriltag ids that you used
        self.used_apriltags = [0, 1, 2, 3, 4, 5, 6, 7, 8]  # add the apriltag ids that you used

        self.transform_aptag_in_cam_dict = {}  # location of apriltags in camera frame
        self.transform_aptag_in_world_dict = {}  # global location of apriltags

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.closest_aptag = None
        self.cam_to_base_link = None

    def publish_tf(self, x, y, z, rot_mat, child_frame_id, frame_id):

        quat = Quaternion()
        quat_ = self.rotation_matrix_to_quaternion(np.array(rot_mat))
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = frame_id
        static_transform.child_frame_id = child_frame_id

        static_transform.transform.translation.x = x  # Set translation values
        static_transform.transform.translation.y = y
        static_transform.transform.translation.z = z  # meters

        static_transform.transform.rotation = quat
        # print('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        self.tf_broadcaster.sendTransform(static_transform)
        print(f'publishing child {child_frame_id} from {frame_id}')

    def get_dist(self, x, y, z):
        return np.linalg.norm([x, y, z])

    def get_transform_matrix_aptags_from_tf(self):
        self.transform_aptag_in_cam_dict = {}
        for aptag in self.used_apriltags:

            str_aptag = str(aptag)
            source_frame = "map"  # to
            # source_frame = "unity"  # to in sim
            frame = "aptag_" + str_aptag  # from

            try:
                print('############ APTAG ########' + str_aptag )
                transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                 timeout=rclpy.duration.Duration(seconds=5.0))

                translation = tr.translation_matrix(
                    [transformation.transform.translation.x, transformation.transform.translation.y,
                     transformation.transform.translation.z])
                rotation = tr.quaternion_matrix(
                    [transformation.transform.rotation.x, transformation.transform.rotation.y,
                     transformation.transform.rotation.z, transformation.transform.rotation.w])
                # Get the homogeneous transformation matrix
                transform_aptag_in_world = np.dot(translation, rotation)

                self.transform_aptag_in_world_dict[aptag] = transform_aptag_in_world

                self.get_logger().info(f'transform ready from {frame} to {source_frame}')

            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
            #     self.get_logger().info('transform not ready')
            #     self.get_logger().info(
            #         f'Could not transform {frame} to {source_frame}')
            #     # raise
            # # print('self.transform_aptag_in_cam_dict', self.transform_aptag_in_world_dict)

    def get_transform_matrix_cam_to_base_link(self):
        frame = "camera_color_optical_frame"  # to
        source_frame = "base_link"  # from

        try:
            # transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
            #                                                  timeout=rclpy.duration.Duration(seconds=5.0))
            ## TODO check why trasnformation from tf broadcast is wrong
            # translation = tr.translation_matrix(
            #     [transformation.transform.translation.x, transformation.transform.translation.y,
            #      transformation.transform.translation.z])

            # rotation = tr.quaternion_matrix(
            #     [transformation.transform.rotation.x, transformation.transform.rotation.y,
            #      transformation.transform.rotation.z, transformation.transform.rotation.w])

            # Get the homogeneous transformation matrix
            # transform_cam_to_base_link = np.dot(translation, rotation)

            transform_cam_to_base_link = np.array([[0.0, 0.0, 1.0, 0.0],
                                                   [1.0, 0.0, 0.0, 0.0],
                                                   [0.0, -1.0, 0.0, 0.0],
                                                   [0.0, 0.0, 0.0, 1.0]])
            self.cam_to_base_link = transform_cam_to_base_link
            quat = Quaternion()
            quat.x = 0.5
            quat.y = -0.5
            quat.z = 0.5
            quat.w = 0.5
            # self.publish_tf( 0.0, 0.0, 0.0, quat, 'camera_color_optical_frame', 'base_link')
            # self.get_logger().info(f'transform ready from {frame} to {source_frame}')
            print('cam_to_base_link', self.cam_to_base_link)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
            self.get_logger().info('transform not ready')
            self.get_logger().info(
                f'Could not transform {frame} to {source_frame}')
            # # raise

    def apriltag_callback(self, msg):
        min_distance = np.inf
        if msg.detections:
            self.transform_aptag_in_cam_dict = {}
            source_frame = msg.header.frame_id  # to
            for at in msg.detections:

                frame = "tag_" + str(at.id)  # from
                # print(frame)
                try:
                    transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                     timeout=rclpy.duration.Duration(seconds=1.0))

                    dist = self.get_dist(transformation.transform.translation.x, transformation.transform.translation.y,
                                         transformation.transform.translation.z)
                    if dist < min_distance:
                        min_distance = dist
                        self.closest_aptag = at.id

                    translation = tr.translation_matrix(
                        [transformation.transform.translation.x, transformation.transform.translation.y,
                         transformation.transform.translation.z])
                    rotation = tr.quaternion_matrix(
                        [transformation.transform.rotation.x, transformation.transform.rotation.y,
                         transformation.transform.rotation.z, transformation.transform.rotation.w])
                    # print('source', source_frame, 'frame', frame, 'rotation', rotation)
                    # Get the homogeneous transformation matrix
                    transform_aptag_in_cam = np.dot(translation, rotation)

                    self.transform_aptag_in_cam_dict[at.id] = transform_aptag_in_cam
                    print('self.transform_aptag_in_cam_dict[at.id]', self.transform_aptag_in_cam_dict[at.id])
                    # self.get_logger().info(f'transform ready from {frame} to {source_frame}')

                except TransformException as ex:

                    # pass
                    self.get_logger().info(
                        f'Could not transform {frame} to {source_frame}: {ex}')
        else:
            # pass
            print('No aptags from callback')

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

    def transform_cam_world_frame(self):
        robot_position = []
        transform_aptag_in_cam_dict = self.transform_aptag_in_cam_dict
        transform_aptag_in_world_dict = self.transform_aptag_in_world_dict

        for aptag in transform_aptag_in_cam_dict.keys():
            # print(aptag)
            t_apriltag_to_world = transform_aptag_in_world_dict[aptag]
            t_apriltag_in_camera = transform_aptag_in_cam_dict[aptag]

            t_cam_in_world = np.dot(t_apriltag_to_world, np.linalg.inv(t_apriltag_in_camera))

            transform_cam_to_base_link = np.array([[0.0, 0.0, 1.0, 0.0],
                                                   [-1.0, 0.0, 0.0, 0.0],
                                                   [0.0, -1.0, 0.0, 0.0],
                                                   [0.0, 0.0, 0.0, 1.0]])
            self.publish_tf(transform_cam_to_base_link[0, 3], transform_cam_to_base_link[1, 3], transform_cam_to_base_link[2, 3], transform_cam_to_base_link[:3,:3],
                            'camera_color_optical_frame', 'base_link_')

            t_robot_in_world = np.dot(t_cam_in_world, transform_cam_to_base_link.T)

            robot_x = t_robot_in_world[0, 3]
            robot_y = t_robot_in_world[1, 3]
            robot_z = t_robot_in_world[2, 3]

            # Append the column vector to the robot_position array
            robot_position.append([robot_x, robot_y, robot_z])

            if aptag == self.closest_aptag:
                # no average for theta jst take the one of the closest aptag
                rotation_matrix = t_robot_in_world[:3, :3]

        # Convert the robot_position list to a NumPy array
        robot_position_array = np.array(robot_position)

        # Compute the mean for each row to get average of position computed from different aptags
        mean_values = np.mean(robot_position_array, axis=0)

        return mean_values, rotation_matrix

    def timer_callback(self):
        # debug
        # self.get_transform_aptags_debug()

        # if apriltags are detected im the scene
        print('self.transform_aptag_in_world_dict', self.transform_aptag_in_world_dict)
        print('ddfgdfrgarswftwewecsd,', self.cam_to_base_link)
        if self.transform_aptag_in_cam_dict and self.transform_aptag_in_world_dict:
            # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
            robot_pose_aptags, rotation_matrix = self.transform_cam_world_frame()

            self.publish_pose(robot_pose_aptags, rotation_matrix)
            self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2], rotation_matrix,
                            'base_link_', 'map')
            # self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2], rotation_matrix,
            #                 'base_link_', 'unity')


        else:
            print('sdjncjds', self.transform_aptag_in_cam_dict)
            if not self.transform_aptag_in_cam_dict:
                self.get_logger().info('NO apriltags detected')
            if not self.transform_aptag_in_world_dict:
                self.get_logger().info('NO transform_aptag_in_world_dict')
            if self.cam_to_base_link is None:
                self.get_logger().info('NO transformation cam_to_base_link')
            # self.get_logger().info('timer_callback failed')

    def publish_pose(self, robot_pose_aptags, rotation_matrix):

        robot_pose = PoseWithCovarianceStamped()

        quat = Quaternion()
        quat_ = self.rotation_matrix_to_quaternion(np.array(rotation_matrix))
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        robot_pose.header.stamp = self.get_clock().now().to_msg()

        robot_pose.header.frame_id = "map"  # or frame_link
        # robot_pose.header.frame_id = "unity"  # or frame_link

        robot_pose.pose.pose.position.x = robot_pose_aptags[0]
        robot_pose.pose.pose.position.y = robot_pose_aptags[1]
        robot_pose.pose.pose.orientation = quat

        self.publisher_.publish(robot_pose)

    # debugging
    def get_transform_aptags_debug(self):
        min_distance = np.inf
        for aptag in self.used_apriltags:

            str_aptag = str(aptag)
            source_frame = "camera_color_optical_frame"  # to
            frame = "tag_" + str_aptag  # from

            try:
                transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                 timeout=rclpy.duration.Duration(seconds=5.0))
                dist = self.dist(transformation.transform.translation.x, transformation.transform.translation.y,
                                 transformation.transform.translation.z)
                if dist < min_distance:
                    min_distance = dist
                    self.closest_aptag = aptag

                translation = tr.translation_matrix(
                    [transformation.transform.translation.x, transformation.transform.translation.y,
                     transformation.transform.translation.z])
                rotation = tr.quaternion_matrix(
                    [transformation.transform.rotation.x, transformation.transform.rotation.y,
                     transformation.transform.rotation.z, transformation.transform.rotation.w])
                # Get the homogeneous transformation matrix

                # Get the homogeneous transformation matrix
                transform_aptag_in_cam = np.dot(translation, rotation)

                self.transform_aptag_in_cam_dict[aptag] = transform_aptag_in_cam
                self.get_logger().info(f'transform ready from {frame} to {source_frame}')

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {frame} to {source_frame}: {ex}')


def main(args=None):
    rclpy.init(args=args)
    get_pose = GetPose()
    # while get_pose.cam_to_base_link is None:
    # get_pose.get_transform_matrix_cam_to_base_link()
    get_pose.get_transform_matrix_aptags_from_tf()
    # print('&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&', get_pose.transform_aptag_in_world_dict )

    rclpy.spin(get_pose)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
