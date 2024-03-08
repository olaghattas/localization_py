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

        # For April tags
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10)
        self.aptags_detected = False
        self.buffer = tf2_ros.Buffer()
        self.tf_buffer = self.buffer
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self, spin_thread=True)
        # self.used_apriltags = [0, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 29]  # add the apriltag ids that you used
        self.used_apriltags = [0, 2, 3, 4, 6, 7, 8, 25, 15, 29, 28, 203]
        # add the apriltag ids that you used
        self.transform_aptag_in_cam_dict = {}  # location of apriltags in camera frame
        self.transform_aptag_in_world_dict = {}  # global location of apriltags
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.closest_aptag = None

        self.result_future = None
        self.time_out = 100
        self.get_tf_info = True
        self.successfully_localized = False
        self.aptags_detected_inside_callback = False

        self.get_transform_matrix_aptags_in_world_from_tf()
        self.timer_period= 2.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.localize)
        self.publisher_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "ola", 10)
        self.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]



    def publish_pose(self, translation, rot_mat):
        robot_pose = PoseWithCovarianceStamped()
        quat_ = self.rotation_matrix_to_quaternion(np.array(rot_mat))
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

    ##### Localization Part #####
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
        self.tf_broadcaster.sendTransform(static_transform)
        # print(f'publishing child {child_frame_id} from {frame_id}')

    def get_dist(self, x, y, z):
        return np.linalg.norm([x, y, z])

    def apriltag_callback(self, msg):
        ### THIS SHOULD HAVE A FLAG IF APRILTAG CALLBACK CALCULATE TF IS TRUE THEN DO THE CLACULATION BUT FIRST IJUST WANT TO CHECK IF THERE ARE TAGS DETECTED

        if msg.detections:
            min_distance = np.inf
            if self.get_tf_info:
                self.transform_aptag_in_cam_dict = {}
                source_frame = msg.header.frame_id  # to
                # source_frame = "base_link"  # to

                try:
                    for at in msg.detections:
                        if at.id == 203:
                            continue
                        self.aptags_detected = True
                        frame = "tag_" + str(at.id)  # from
                        transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                         timeout=rclpy.duration.Duration(
                                                                             seconds=1000.0))

                        dist = self.get_dist(transformation.transform.translation.x,
                                             transformation.transform.translation.y,
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
                        # print('self.transform_aptag_in_cam_dict[at.id]', self.transform_aptag_in_cam_dict[at.id])
                        # self.get_logger().info(f'transform ready from {frame} to {source_frame}')
                        # print("aptag detected!!!!!!")
                        self.aptags_detected_inside_callback = True
                        # print('********** apriltag detectedd: ' + str(at.id))

                except Exception as ex:
                    # self.aptags_detected_inside_callback = False
                    self.get_logger().info(f'Error ***************: {ex}')

        else:
            # pass
            print('No aptags from callback')
            self.aptags_detected = False
            self.aptags_detected_inside_callback = False

    def get_transform_matrix_aptags_in_world_from_tf(self):
        self.transform_aptag_in_cam_dict = {}
        for aptag in self.used_apriltags:

            str_aptag = str(aptag)
            source_frame = "map"  # to
            frame = "aptag_" + str_aptag  # from

            try:
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

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

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

    def particles_callback(self, msg):
        max_weight = 0.0  # Initialize with a double value
        particles_count = 0.0

        for particle in msg.particles:
            weight = particle.weight

            if weight > max_weight:
                max_weight = weight
                particles_count += particles_count

        self.max_weight = max_weight

        if particles_count > 0:
            self.get_logger().info('no particles')

        # print(self.max_weight, '**************************')

    def transform_cam_world_frame(self):
        robot_position = []
        transform_aptag_in_cam_dict = self.transform_aptag_in_cam_dict
        transform_aptag_in_world_dict = self.transform_aptag_in_world_dict

        for aptag in transform_aptag_in_cam_dict.keys():
            # print(aptag)te
            t_apriltag_to_world = transform_aptag_in_world_dict[aptag]
            t_apriltag_in_camera = transform_aptag_in_cam_dict[aptag]

            t_cam_in_world = np.dot(t_apriltag_to_world, np.linalg.inv(t_apriltag_in_camera))

            ## get it from  ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link
            transform_cam_to_base_link = np.array([[-0.397, -0.062, -0.916,  1.212],
                                                   [-0.225,  0.974,  0.031, -0.033],
                                                   [0.890,  0.218, -0.401,  0.492],
                                                   [0.0, 0.0, 0.0, 1.0]])

            ## get it from  ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
            # transform_cam_to_base_link = np.array([[-0.394, -0.217, 0.893, 0.036],
            #                                       [-0.041,  0.975,  0.219,  0.001],
            #                                       [-0.918,  0.050, -0.393,  1.308],
            #                                       [0.0, 0.0, 0.0, 1.0]])


            t_robot_in_world = np.dot(t_cam_in_world, transform_cam_to_base_link)

            # t_robot_in_world = np.dot(t_cam_in_world, np.linalg.inv(transform_cam_to_base_link))

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

    def localize(self):
        # LOCALIZE
        start_time = time.time()
        speed = 3.14 / 8.0
        msg = Twist()
        msg.angular.z = speed

        # CHECK IF THERE ARE ANY APRILTAGS
        if not self.aptags_detected:
            # rotate until you find one
            while time.time() - start_time < self.time_out:
                self.get_logger().info('no aptags detected will start looking for one')
                # change this to VECTOR FIELD HISTOGRAM exploration
                # TODO: uncomment
                self.vel_pub.publish(msg)
                if self.aptags_detected_inside_callback:
                    # STOP
                    msg.angular.z = 0.0
                    # TODO: uncomment
                    self.vel_pub.publish(msg)
                    # localize
                    self.get_tf_info = True
                    # print("published_pose)))))))))))))))))))" )
                    if self.transform_aptag_in_cam_dict and self.transform_aptag_in_world_dict:
                        # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
                        robot_pose_aptags, rotation_matrix = self.transform_cam_world_frame()

                        self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2],
                                        rotation_matrix,
                                        'base_link_ola', 'map')
                        self.successfully_localized = True

                        return

                    else:
                        # self.successfully_localized = False
                        if not self.transform_aptag_in_cam_dict:
                            self.get_logger().info('NO apriltags detected')
                        if not self.transform_aptag_in_world_dict:
                            self.get_transform_matrix_aptags_in_world_from_tf()
                            self.get_logger().info('NO transform_aptag_in_world_dict')
                        # if self.cam_to_base_link is None:
                        #     self.get_logger().info('NO transformation cam_to_base_link')
                        # return
        else:
            # localize
            self.get_tf_info = True
            # print("published_pose)))))))))))))))))))" )
            if self.transform_aptag_in_cam_dict and self.transform_aptag_in_world_dict:
                # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
                robot_pose_aptags, rotation_matrix = self.transform_cam_world_frame()

                # print("published_pose")
                self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2], rotation_matrix,
                                'base_link_ola', 'map')
                self.publish_pose(robot_pose_aptags, rotation_matrix)

                self.successfully_localized = True

                return

            else:
                # self.successfully_localized = False

                if not self.transform_aptag_in_cam_dict:
                    self.get_logger().info('NO apriltags detected')
                if not self.transform_aptag_in_world_dict:
                    self.get_transform_matrix_aptags_in_world_from_tf()
                    self.get_logger().info('NO transform_aptag_in_world_dict')
                # return

                # if self.cam_to_base_link is None:
                #     self.get_logger().info('NO transformation cam_to_base_link')


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
