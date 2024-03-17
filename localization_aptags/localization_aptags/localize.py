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
from apriltag_msgs.msg import AprilTagDetectionArray


class LocalizationActionServer(Node):

    def __init__(self):
        super().__init__('Localization_action_server')

        # For localization

        self.vel_pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 10)

        # add the apriltag ids that you used
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.timer_period = 2.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.localize)

        self.publisher_gt = self.create_publisher(PoseWithCovarianceStamped, "ground_truth", 10)
        self.publisher_matrixmul = self.create_publisher(PoseWithCovarianceStamped, "matrix_cam", 10)
        self.publisher_gt_2tf = self.create_publisher(PoseWithCovarianceStamped, "ground_truth_jjdj", 10)
        self.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.06853892326654787]

        self.buffer = tf2_ros.Buffer()
        self.tf_buffer = self.buffer
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.transform_aptag_in_world = None
        self.transform_cam_in_aptag = None
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)

    def get_pose(self, translation, quat_):
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

        return robot_pose

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
        static_transform.transform.translation.z = translation[2]  # meters

        static_transform.transform.rotation = quat
        self.tf_broadcaster.sendTransform(static_transform)
        # print(f'publishing child {child_frame_id} from {frame_id}')

    def localize(self):
        # quat = [0.0,0.0, 0.0, 1.0]
        # translation = [-1.1,2.1,0.0]
        # # self.publish_pose(translation, quat)
        # # self.publish_tf(translation, quat, "base_link_","map")
        # translation = [0.833, -0.258, 1.309]
        # # # - Rotation: in Quaternion [0.000, 0.000, 0.997, -0.071]
        # # #[-0.430, -0.073, 0.000] #[[-1.237, 0.278, 0.000]
        # quaternion = [0.204, 0.806, -0.118, 0.543] #[0.000, 0.000, -0.343, 0.939] #[0.000, 0.000, 0.876, -0.482] #[0.000, 0.000, 0.876, 0.482]

        # translation = [0.805, -0.235, 0.000]
        # # - Rotation: in Quaternion [0.000, 0.000, -0.343, 0.939][1.533, -0.193, -0.295]
        # quaternion = [0.000, 0.000, -0.343, 0.939] #[-0.204, -0.806, 0.118, 0.543]
        #
        # robot_pose_gt = self.get_pose(translation, quaternion)
        # self.publisher_gt_2tf.publish(robot_pose_gt)
        # print("localizejfnasdklfbhsdgb")
        self.get_transf()

    def apriltag_callback(self, msg):
        ### THIS SHOULD HAVE A FLAG IF APRILTAG CALLBACK CALCULATE TF IS TRUE THEN DO THE CLACULATION BUT FIRST IJUST WANT TO CHECK IF THERE ARE TAGS DETECTED
        print("aptag callbacl")
        if msg.detections:
            frame = msg.header.frame_id  # to
            # source_frame = "base_link"  # to

            try:
                source_frame = "tag_15"  # from
                transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                 timeout=rclpy.duration.Duration(
                                                                     seconds=1000.0))

                translation = tr.translation_matrix(
                    [transformation.transform.translation.x, transformation.transform.translation.y,
                     transformation.transform.translation.z])
                rotation = tr.quaternion_matrix(
                    [transformation.transform.rotation.x, transformation.transform.rotation.y,
                     transformation.transform.rotation.z, transformation.transform.rotation.w])
                # print('source', source_frame, 'frame', frame, 'rotation', rotation)
                # Get the homogeneous transformation matrix
                self.transform_cam_in_aptag = np.dot(translation, rotation)




                transformation = self.tf_buffer.lookup_transform(frame, "base_link", rclpy.time.Time(),
                                                                 timeout=rclpy.duration.Duration(
                                                                     seconds=1000.0))
                translation = tr.translation_matrix(
                    [transformation.transform.translation.x, transformation.transform.translation.y,
                     transformation.transform.translation.z])
                rotation = tr.quaternion_matrix(
                    [transformation.transform.rotation.x, transformation.transform.rotation.y,
                     transformation.transform.rotation.z, transformation.transform.rotation.w])
                # print('source', source_frame, 'frame', frame, 'rotation', rotation)
                # Get the homogeneous transformation matrix
                self.transform_base_in_cam = np.dot(translation, rotation)

            except Exception as ex:
                # self.aptags_detected_inside_callback = False
                self.get_logger().info(f'Error ***************: {ex}')

    def map_tag(self):
        ### THIS SHOULD HAVE A FLAG IF APRILTAG CALLBACK CALCULATE TF IS TRUE THEN DO THE CLACULATION BUT FIRST IJUST WANT TO CHECK IF THERE ARE TAGS DETECTED

        source_frame = "map"  # to
        frame = "aptag_15"

        transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                         timeout=rclpy.duration.Duration(
                                                             seconds=1000.0))
        translation = tr.translation_matrix(
            [transformation.transform.translation.x, transformation.transform.translation.y,
             transformation.transform.translation.z])
        rotation = tr.quaternion_matrix(
            [transformation.transform.rotation.x, transformation.transform.rotation.y,
             transformation.transform.rotation.z, transformation.transform.rotation.w])
        # print('source', source_frame, 'frame', frame, 'rotation', rotation)
        # Get the homogeneous transformation matrix
        self.transform_aptag_in_world = np.dot(translation, rotation)





    def get_transf(self):
        print("GET tRANS")
        self.map_tag()
        if self.transform_aptag_in_world is not None and self.transform_cam_in_aptag is not None:
            self.cam_in_map = self.transform_aptag_in_world @ (self.transform_cam_in_aptag @ self.transform_base_in_cam)
            print("self.cam_in_map", self.cam_in_map)

            quat_ = self.rotation_matrix_to_quaternion(np.array(self.cam_in_map[:3, :3]))

            robo = self.get_pose([self.cam_in_map[0, 3], self.cam_in_map[1, 3], self.cam_in_map[2, 3]],
                                 [quat_[0], quat_[1], quat_[2], quat_[3]])
            self.publisher_matrixmul.publish(robo)

            ##################
        frame = "base_link"
        source_frame = "map"
        transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                         timeout=rclpy.duration.Duration(
                                                             seconds=1000.0))
        print("1555555")
        # translation = tr.translation_matrix(
        #    )
        # rotation = tr.quaternion_matrix(
        #    )
        # print('source', source_frame, 'frame', frame, 'rotation', rotation)
        # Get the homogeneous trans [transformation.transform.rotation.x, transformation.transform.rotation.y,
        #             #      transformation.transform.rotation.z, transformation.transform.rotation.w]formation matrix
        # self.transform_cam_in_world= np.dot(translation, rotation)
        robot = self.get_pose([transformation.transform.translation.x, transformation.transform.translation.y,
                               transformation.transform.translation.z],
                              [transformation.transform.rotation.x, transformation.transform.rotation.y,
                               transformation.transform.rotation.z, transformation.transform.rotation.w])
        self.publisher_gt.publish(robot)



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
