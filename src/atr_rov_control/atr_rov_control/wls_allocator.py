import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
from scipy.optimize import minimize


# B matrix: Phase 1 Section 7.2
B = np.array([
    [ 0,      0,      0,      0,      1,      1,      0,      0     ],
    [ 0,      0,      0,      0,      0,      0,     -1,     -1     ],
    [ 1,      1,      1,      1,      0,      0,      0,      0     ],
    [-0.1536, 0.1404,-0.1539, 0.1401, 0,      0,     -0.1819,-0.1829],
    [-0.3714,-0.3714, 0.5389, 0.5389,-0.1217,-0.1210, 0,      0     ],
    [ 0,      0,      0,      0,     -0.1334, 0.1446, 0.3848,-0.1912],
])


class WLSAllocator(Node):
    def __init__(self):
        super().__init__('wls_allocator')

        self.declare_parameter('f_min', [-200.]*4 + [-100.]*4)
        self.declare_parameter('f_max', [ 200.]*4 + [ 100.]*4)
        self.declare_parameter('w_tau', [1e6, 1e6, 1e6, 0.0, 1e6, 1e6])
        self.declare_parameter('w_f',   [1.0]*8)

        self.sub = self.create_subscription(
            Float64MultiArray, '/control/tau', self.tau_cb, 10)

        # Publishers: one per thruster
        self.pubs = [
            self.create_publisher(
                Float64, f'/atr_mp_rov/thrusters/t{i+1}/force', 10)
            for i in range(8)]

    def tau_cb(self, msg):
        tau = np.array(msg.data)  # [X, Y, Z, K, M, N]
        if len(tau) != 6:
            return

        f_min = np.array(self.get_parameter('f_min').value)
        f_max = np.array(self.get_parameter('f_max').value)
        w_tau = np.array(self.get_parameter('w_tau').value)
        w_f   = np.array(self.get_parameter('w_f').value)

        W_tau = np.diag(w_tau)
        W_f   = np.diag(w_f)

        # WLS objective: (Bf - tau)'W_tau(Bf - tau) + f'W_f f
        def objective(f):
            residual = B @ f - tau
            return residual @ W_tau @ residual + f @ W_f @ f

        def gradient(f):
            residual = B @ f - tau
            return 2 * B.T @ W_tau @ residual + 2 * W_f @ f

        # Warm start: pseudoinverse solution
        f0 = np.linalg.pinv(B) @ tau
        f0 = np.clip(f0, f_min, f_max)

        bounds = list(zip(f_min, f_max))
        result = minimize(
            objective, f0, jac=gradient,
            method='L-BFGS-B', bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-9})

        f_alloc = np.clip(result.x, f_min, f_max)

        # Publish to all 8 thrusters
        for i, pub in enumerate(self.pubs):
            msg_out = Float64()
            msg_out.data = float(f_alloc[i])
            pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    wls_allocator = WLSAllocator()
    rclpy.spin(wls_allocator)
    wls_allocator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()