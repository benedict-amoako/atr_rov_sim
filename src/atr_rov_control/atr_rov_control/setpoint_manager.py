import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import math

class SetpointManager(Node):
    def __init__(self):
        super().__init__('setpoint_manager')

        # Setpoint ramp rate (m/s for postition, rad/s for orientation)
        self.declare_parameter('ramp_rate_linear', 0.5) # m/s
        self.declare_parameter('ramp_rate_angular', 0.2) # rad/s
        self.declare_parameter('control_rate', 50.0) # Hz

        # Current active setpoints
        self.pos_active = PoseStamped()
        self.pos_active.pose.orientation.w = 1.0  # Default orientation (no rotation)
        self.vel_active = TwistStamped()

        # Desired setpoints (from external commands)
        self.pos_desired = PoseStamped()
        self.pos_desired.pose.orientation.w = 1.0  # Default orientation (no rotation)
        self.vel_desired = TwistStamped()

        self.sub_pos = self.create_subscription(PoseStamped, '/control/setpoint/position', self.pos_cb, 10)
        self.sub_vel = self.create_subscription(TwistStamped, '/control/setpoint/velocity', self.vel_cb, 10)

        self.pub_pos = self.create_publisher(PoseStamped, '/control/setpoint/position_active', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/control/setpoint/velocity_active', 10)

        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self.update)

    def pos_cb(self, msg):
        self.pos_desired = msg

    def vel_cb(self, msg):
        self.vel_desired = msg

    def update(self):
        dt = 1.0 / self.get_parameter('control_rate').value
        r_lin = self.get_parameter('ramp_rate_linear').value * dt
        r_ang = self.get_parameter('ramp_rate_angular').value * dt

        # Ramp position setpoint toward desired position
        self.pos_active.pose.position.x = self._ramp(self.pos_active.pose.position.x, self.pos_desired.pose.position.x, r_lin)
        self.pos_active.pose.position.y = self._ramp(self.pos_active.pose.position.y, self.pos_desired.pose.position.y, r_lin)
        self.pos_active.pose.position.z = self._ramp(self.pos_active.pose.position.z, self.pos_desired.pose.position.z, r_lin)

        self.pos_active.header.stamp = self.get_clock().now().to_msg()
        self.pos_active.header.frame_id = 'ned_odom'
        self.pub_pos.publish(self.pos_active)

        self.vel_active.header.stamp = self.get_clock().now().to_msg()
        self.vel_active.header.frame_id = 'base_link'
        self.pub_vel.publish(self.vel_active)

    @staticmethod
    def _ramp(current, desired, max_step):
        delta = desired - current
        if abs(delta) <= max_step:
            return desired
        return current  + math.copysign(max_step, delta)
    
def main(args=None):
    rclpy.init(args=args)
    setpoint_manager = SetpointManager()
    rclpy.spin(setpoint_manager)
    setpoint_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
