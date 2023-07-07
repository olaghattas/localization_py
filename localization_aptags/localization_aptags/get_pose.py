import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from time import time
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

class GetPose(Node):

    def __init__(self):
        super().__init__('get_pose')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, "robot_pose", 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(AprilTagDetectionArray, '/detections', self.apriltag_callback, 10)
        self.subscription  # to prevent unused variable warning


    def apriltag_callback(self, msg):
        if msg.detections:
cd



    def transform_cam_world_frame(self):
        pass

    def timer_callback(self):
        robot_pose = PoseWithCovarianceStamped()

        current_time = time()
        robot_pose.header.stamp.sec = int(current_time)
        robot_pose.header.stamp.nanosec = int((current_time - int(current_time)) * 1e9)
        robot_pose.header.frame_id = "map" # or frame_link

        robot_pose.pose.pose.position.x = 0
        robot_pose.pose.pose.position.y = 0
        robot_pose.pose.pose.orientation.w = 0


def main(args=None):

    rclpy.init(args=args)
    get_pose = GetPose()
    rclpy.spin(get_pose)


if __name__ == '__main__':
    main()
