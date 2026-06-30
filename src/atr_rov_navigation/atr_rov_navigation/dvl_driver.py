import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

class DVLDriver(Node):
    def __init__(self):
        super().__init__('dvl_driver')

        # DVL velocity noise std dev(m/s) per axis
        self.declare_parameter('dvl_noise_std', 0.02)

        self.sub = self.create_subscription(
            Odometry,
            '/nav/ins_odom',
            self.odom_cb, 10)
        
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/dvl/velocity', 10)

    def odom_cb(self, msg):
        noise = self.get_parameter('dvl_noise_std').value
        var = noise ** 2

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = msg.header.stamp
        twist_msg.header.frame_id = 'dvl_link'

        # Pass through linear velocity from odometry ground truth
        # In Phase 5: replace with physical DVL serial/ethernet driver
        twist_msg.twist.twist.linear = msg.twist.twist.linear

        # diagonal covariance: vx. vy, vz variance
        # 6x6 matrix stored as flat array pf 36 elements
        twist_msg.twist.covariance[0] = var  # vx variance
        twist_msg.twist.covariance[7] = var  # vy variance
        twist_msg.twist.covariance[14] = var  # vz variance

        self.pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DVLDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()