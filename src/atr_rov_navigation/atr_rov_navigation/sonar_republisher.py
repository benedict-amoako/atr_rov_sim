import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SonarRepublisher(Node):
    def __init__(self):
        super().__init__('sonar_republisher')

        self.sub = self.create_subscription(
            LaserScan,
            '/sonar/scan',
            self.sonar_cb, 10)
        
        self.pub = self.create_publisher(LaserScan, '/sonar/scan_fixed', 10)

    def sonar_cb(self, msg):
        out = LaserScan()
        out.header.stamp = msg.header.stamp

        # Fix frame ID from Ignition internal path to standard link name
        out.header.frame_id = 'sonar_link'

        # Pass through measurement data unchanged
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SonarRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()