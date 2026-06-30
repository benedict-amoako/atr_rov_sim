import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMURepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')

        self.sub = self.create_subscription(
            Imu,
            '/ins/imu',
            self.imu_cb, 10)
        
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

    def imu_cb(self, msg):
        out = Imu()
        out.header.stamp = msg.header.stamp

        # Fix frame ID from Ignition internal path to standard link name
        out.header.frame_id = 'ins_link'

        # Pass through measurement data unchnaged
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        # Orientation covariance (rad^2) - diagonal 3x3 stored as flat 9-element array
        out.orientation_covariance[0] = 0.001  # roll variance
        out.orientation_covariance[4] = 0.001  # pitch variance
        out.orientation_covariance[8] = 0.001  # yaw variance

        # Angular velocity corariance (rad^2/s^2)
        # Noise std = 0.001 rad/s from xacro -> variance = 1e-6
        out.angular_velocity_covariance[0] = 1e-6  # roll rate variance
        out.angular_velocity_covariance[4] = 1e-6  # pitch rate variance
        out.angular_velocity_covariance[8] = 1e-6  # yaw rate variance

        # Linear acceleration covariance (m^2/s^4)
        # Noise std = 0.01 m/s^2 from xacro -> variance = 1e-4
        out.linear_acceleration_covariance[0] = 1e-4  # x acceleration variance
        out.linear_acceleration_covariance[4] = 1e-4  # y acceleration variance
        out.linear_acceleration_covariance[8] = 1e-4  # z acceleration variance

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = IMURepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()