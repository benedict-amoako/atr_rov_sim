import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped

class DepthDriver(Node):
    def __init__(self):
        super().__init__('depth_driver')

        # Parameters
        self.declare_parameter('depth_noise_std', 0.05)  # Standard deviation of depth noise in meters

        # Subscribe to odometry instead of pressure sensor
        self.sub = self.create_subscription(
            Odometry,
            '/nav/ins_odom',
            self.odom_cb, 10)
        
        self.pub_depth = self.create_publisher(Float64, '/depth/metres', 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/depth/pose', 10)

    def odom_cb(self, msg):
        noise = self.get_parameter('depth_noise_std').value

        # In ENU, Z is positive upward. Depth = -Z from spawn surface.
        # The ROV spawns at z=-2.0 in ENU world frame.
        # Depth in NED = distance below surface = -z_enu
        enu_z = msg.pose.pose.position.z
        depth = -enu_z  # convert ENU Z to NED depth (positive downward)
        depth = max(0.0, depth)  # Ensure depth is not negative

        # Publish raw depth in metres
        depth_msg = Float64()
        depth_msg.data = depth
        self.pub_depth.publish(depth_msg)

        # Publish depth as PoseWithCovarianceStamped for EKF
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'ned_odom'
        # NED: depth is positive Z downward
        pose_msg.pose.pose.position.z = depth
        pose_msg.pose.pose.orientation.w = 1.0  # No rotation
        # 6x6 covariance matrix, only Z position variance is set
        pose_msg.pose.covariance[14] = noise ** 2  # index 14 is Z position variance
        self.pub_pose.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()