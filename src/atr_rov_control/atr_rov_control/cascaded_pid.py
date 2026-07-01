import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
import math
from .pid_utils import PID
from tf_transformations import euler_from_quaternion

class CascadedPID(Node):
    def __init__(self):
        super().__init__('cascaded_pid')
        self._declare_params()
        self._init_pids()

        # Subscriptions
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.sub_pos_sp = self.create_subscription(PoseStamped, '/control/setpoint/position_active', self.pos_sp_cb, 10)
        self.sub_vel_ff = self.create_subscription(TwistStamped, '/control/setpoint/velocity_active', self.vel_ff_cb, 10)

        # Publisher: generalised force vector tau [X, Y, Z, K, M, N]
        self.pub_tau = self.create_publisher(Float64MultiArray, '/control/tau', 10)

        # State
        self.odom = None
        self.pos_sp = PoseStamped()
        self.vel_ff = TwistStamped()
        self.prev_time = None

        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

    def _declare_params(self):
        params = [
            'control_rate',  # Hz
            'outer_heave_kp', 'outer_heave_ki', 'outer_heave_kd',
            'outer_surge_kp', 'outer_surge_ki', 'outer_surge_kd',
            'outer_sway_kp', 'outer_sway_ki', 'outer_sway_kd',
            'outer_pitch_kp', 'outer_pitch_ki', 'outer_pitch_kd',
            'outer_yaw_kp', 'outer_yaw_ki', 'outer_yaw_kd',
            'inner_heave_kp', 'inner_heave_ki', 'inner_heave_kd',
            'inner_surge_kp', 'inner_surge_ki', 'inner_surge_kd',
            'inner_sway_kp', 'inner_sway_ki', 'inner_sway_kd',
            'inner_pitch_kp', 'inner_pitch_ki', 'inner_pitch_kd',
            'inner_yaw_kp', 'inner_yaw_ki', 'inner_yaw_kd',
            'outer_velocity_limit',
            'inner_force_limit_xyz',
            'inner_torque_limit_mn',
            'integral_limit_xyz',
            'integral_limit_mn'
        ]
        for param in params:
            self.declare_parameter(param, 0.0)  # Default to 0.0 for all parameters

    def _init_pids(self):
        lim_v = self.get_parameter('outer_velocity_limit').value
        lim_f = self.get_parameter('inner_force_limit_xyz').value
        lim_t = self.get_parameter('inner_torque_limit_mn').value
        ilim_f = self.get_parameter('integral_limit_xyz').value
        ilim_t = self.get_parameter('integral_limit_mn').value

        def outer(dof):
            return PID(
                kp=self.get_parameter(f'outer_{dof}_kp').value,
                ki=self.get_parameter(f'outer_{dof}_ki').value,
                kd=self.get_parameter(f'outer_{dof}_kd').value,
                output_min=-lim_v, output_max=lim_v
            )
        
        def inner(dof, lim, ilim):
            return PID(
                kp=self.get_parameter(f'inner_{dof}_kp').value,
                ki=self.get_parameter(f'inner_{dof}_ki').value,
                kd=self.get_parameter(f'inner_{dof}_kd').value,
                output_min=-lim, output_max=lim,
                integral_min=-ilim, integral_max=ilim
            )
        
        self.outer = {
            'heave': outer('heave'), 'surge': outer('surge'), 
            'sway': outer('sway'), 'pitch': outer('pitch'), 
            'yaw': outer('yaw')}
        self.inner = {
            'heave': inner('heave', lim_f, ilim_f), 
            'surge': inner('surge', lim_f, ilim_f),
            'sway': inner('sway', lim_f, ilim_f), 
            'pitch': inner('pitch', lim_t, ilim_t),
            'yaw': inner('yaw', lim_t, ilim_t)}
        
    def odom_cb(self, msg):
        self.odom = msg

    def pos_sp_cb(self, msg):
        self.pos_sp = msg

    def vel_ff_cb(self, msg):
        self.vel_ff = msg

    def control_loop(self):
        if self.odom is None:
            return  # Wait until odometry is received
        
        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0.0:
            return  # Avoid division by zero or negative time step
        
        # Current state from EKF
        pos = self.odom.pose.pose.position
        vel = self.odom.twist.twist.linear
        ang = self.odom.twist.twist.angular
        q = self.odom.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Setpoints
        sp_pos = self.pos_sp.pose.position
        sp_q = self.pos_sp.pose.orientation
        _, sp_pitch, sp_yaw = euler_from_quaternion([sp_q.x, sp_q.y, sp_q.z, sp_q.w])

        # Velocity feedforward
        ff_vel = self.vel_ff.twist

        # Outer loop: position error -> velocity demand
        vz_demand = self.outer['heave'].update(sp_pos.z - pos.z, dt) + ff_vel.linear.z
        vx_demand = self.outer['surge'].update(sp_pos.x - pos.x, dt) + ff_vel.linear.x
        vy_demand = self.outer['sway'].update(sp_pos.y - pos.y, dt) + ff_vel.linear.y
        vp_demand = self.outer['pitch'].update(sp_pitch - pitch, dt) + ff_vel.angular.y
        vn_demand = self.outer['yaw'].update(self._angle_error(sp_yaw, yaw), dt) + ff_vel.angular.z

        # Inner loop: velocity error -> generalized force/torque demand
        tau_z = self.inner['heave'].update(vz_demand - vel.z, dt)
        tau_x = self.inner['surge'].update(vx_demand - vel.x, dt)
        tau_y = self.inner['sway'].update(vy_demand - vel.y, dt)
        tau_m = self.inner['pitch'].update(vp_demand - ang.y, dt)
        tau_n = self.inner['yaw'].update(vn_demand - ang.z, dt)
        tau_k = 0.0 # Roll: passive stabilization, no control

        # Publish tau = [X, Y, Z, K, M, N]
        tau_msg = Float64MultiArray()
        tau_msg.data = [tau_x, tau_y, tau_z, tau_k, tau_m, tau_n]
        self.pub_tau.publish(tau_msg)

    @staticmethod
    def _angle_error(desired, actual):
        """Shortest angular distance, wrapped to [-pi, pi]"""
        err = desired - actual
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        return err
    
def main(args=None):
    rclpy.init(args=args)
    node = CascadedPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()