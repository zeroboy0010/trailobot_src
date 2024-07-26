import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
import numpy as np
import casadi as ca
from casadi import sin, cos, pi
import math
# setting matrix_weights' variables
Q_x = 13
Q_y = 150
Q_theta = 3

############
R1 = 1
R2 = 1

step_horizon = 0.05  # time between steps in seconds
N = 10              # number of look ahead steps
N_IND_SEARCH = N
## Q


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return yaw_z # in radians


############ function ###############
def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value
#############

def DM2Arr(dm):
    return np.array(dm.full())

def calc_nearest_index(current_x, current_y, cx, cy, cyaw, pind):

    dx = [current_x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [current_y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - current_x
    dyl = cy[ind] - current_y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

class mpc_class(Node):
    def __init__(self):
        super().__init__('mpc_node_test')
        self.mpc_timer = self.create_timer(step_horizon, self.mpc_callback)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription1 = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.listener_callback,
            20)

        self.subscription3 = self.create_subscription(
            Odometry,
            '/odom',
            self.feedback_callback,
            30)
        self.V_pub = [0.0, 0.0]
        self.x_pos = 0
        self.y_pos = 0
        self.omega_pos = 0
        self.control = [0.0,0.0]

        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
                ##### mpc #####
        # specs
        self.x_init = 0.0
        self.y_init = 0.0
        self.theta_init = 0.0
        self.x_target = 0.0
        self.y_target = 0.0
        self.theta_target = 0.0

        self.phi_test = 0
        ## mpc start
        self.slow_speed = 1.0
        self.v_max = 5.0
        self.start_mpc()
        ## 
        self.goal = False
        ## 
        self.controller_state = True

        
    def listener_callback(self, msg):
        # msg = PoseStamped()
        # msg.pose.orientation.w
        self.x_target = msg.pose.position.x
        self.y_target = msg.pose.position.y
        self.theta_target = euler_from_quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        self.state_target = ca.DM([self.x_target, self.y_target, self.theta_target])  # target state

    def feedback_callback(self, msg):
        # msg = Odometry()
        self.current_x = (float)(msg.pose.pose.position.x)
        self.current_y = (float)(msg.pose.pose.position.y)

        self.current_yaw = (float)(euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))
        self.state_init = ca.DM([self.current_x, self.current_y, self.current_yaw])
        # print(self.current_x,"::",self.current_y,"::",self.current_yaw)
        # print(self.v_max)


    def start_mpc(self):
        # state symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(
            x,
            y,
            theta
        )
        self.n_states = states.numel()

        # control symbolic variables
        V = ca.SX.sym('V')
        Omega = ca.SX.sym('Omega')
        controls = ca.vertcat(
            V,
            Omega
        )
        controls_2 = ca.vertcat(
            V,
            0.0,
            Omega
        )
        self.n_controls = controls.numel()

        # matrix containing all states over all time steps +1 (each column is a state vector)
        X = ca.SX.sym('X', self.n_states, N + 1)

        # matrix containing all control actions over all time steps (each column is an action vector)
        U = ca.SX.sym('U', self.n_controls, N)

        # coloumn vector for storing initial state and target state
        P = ca.SX.sym('P', self.n_states + self.n_states)

        # state weights matrix (Q_X, Q_Y, Q_THETA)
        Q = ca.diagcat(Q_x, Q_y, Q_theta)

        # controls weights matrix
        R = ca.diagcat(R1, R2)

        # discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
        rot_3d_z = ca.vertcat(
            ca.horzcat(cos(theta), -sin(theta), 0),
            ca.horzcat(sin(theta),  cos(theta), 0),
            ca.horzcat(         0,           0, 1)
        )

        RHS = rot_3d_z @  controls_2
        # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
        self.f = ca.Function('f', [states, controls], [RHS])


        cost_fn = 0  # cost function
        g = X[:, 0] - P[:self.n_states]  # constraints in the equation


        # runge kutta
        for k in range(N):
            st = X[:, k]
            con = U[:, k]
            cost_fn = cost_fn \
                + (st - P[self.n_states:]).T @ Q @ (st - P[self.n_states:]) \
                + con.T @ R @ con
            st_next = X[:, k+1]
            k1 = self.f(st, con)
            k2 = self.f(st + step_horizon/2*k1, con)
            k3 = self.f(st + step_horizon/2*k2, con)
            k4 = self.f(st + step_horizon * k3, con)
            st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g = ca.vertcat(g, st_next - st_next_RK4)


        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )
        nlp_prob = {
            'f': cost_fn,
            'x': OPT_variables,
            'g': g,
            'p': P
        }

        opts = {
            'ipopt': {
                'max_iter': 2000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }

        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        self.lbx = ca.DM.zeros((self.n_states*(N+1) + self.n_controls*N, 1))
        self.ubx = ca.DM.zeros((self.n_states*(N+1) + self.n_controls*N, 1))

        self.lbx[0: self.n_states*(N+1): self.n_states] = -ca.inf     # X lower bound
        self.lbx[1: self.n_states*(N+1): self.n_states] = -ca.inf     # Y lower bound
        self.lbx[2: self.n_states*(N+1): self.n_states] = -ca.inf     # theta lower bound

        self.ubx[0: self.n_states*(N+1): self.n_states] = ca.inf      # X upper bound
        self.ubx[1: self.n_states*(N+1): self.n_states] = ca.inf      # Y upper bound
        self.ubx[2: self.n_states*(N+1): self.n_states] = ca.inf      # theta upper bound

        self.lbx[self.n_states*(N+1)    : self.n_states*(N+1) + self.n_controls*N : self.n_controls] = -1.0               # v lower bound for all V
        self.lbx[self.n_states*(N+1) + 1: self.n_states*(N+1) + self.n_controls*N : self.n_controls] = -0.5                  # omega low

        self.ubx[self.n_states*(N+1)    : self.n_states*(N+1) + self.n_controls*N : self.n_controls] = 1.0               # v lower bound for all V
        self.ubx[self.n_states*(N+1) + 1: self.n_states*(N+1) + self.n_controls*N : self.n_controls] = 0.5            # omega low

        self.state_init = ca.DM([self.x_init, self.y_init, self.theta_init])        # initial state
        self.state_target = ca.DM([self.x_target, self.y_target, self.theta_target])  # target state

        self.u0 = ca.DM.zeros((self.n_controls, N))  # initial control
        self.X0 = ca.repmat(self.state_init, 1, N+1)         # initial state full

        self.t = 0
        
    ####################################

    def mpc_callback(self):
        pub_msg = Twist()
        if ((ca.norm_2(self.state_init - self.state_target)) > 1e-3):
            self.args = {
                'lbg': ca.DM.zeros((self.n_states*(N+1), 1)),  # constraints lower bound
                'ubg': ca.DM.zeros((self.n_states*(N+1), 1)),  # constraints upper bound
                'lbx': self.lbx,
                'ubx': self.ubx
            }
            self.args['p'] = ca.vertcat(
                self.state_init,    # current state
                self.state_target   # target state
            )
            # optimization variable current state
            self.args['x0'] = ca.vertcat(
                ca.reshape(self.X0, self.n_states*(N+1), 1),
                ca.reshape(self.u0, self.n_controls*N, 1)
            )

            sol = self.solver(
                x0=self.args['x0'],
                lbx=self.args['lbx'],
                ubx=self.args['ubx'],
                lbg=self.args['lbg'],
                ubg=self.args['ubg'],
                p=self.args['p']
            )

            u = ca.reshape(sol['x'][self.n_states * (N + 1):], self.n_controls, N)
            self.X0 = ca.reshape(sol['x'][: self.n_states * (N+1)], self.n_states, N+1)

            self.control = u[:,0]
            
            self.X0 = ca.horzcat(
                self.X0[:, 1:],
                ca.reshape(self.X0[:, -1], -1, 1)
            )
            self.u0 = ca.horzcat(
                u[:, 1:],
                ca.reshape(u[:, -1], -1, 1)
            )

        else :
            self.V_pub = [0,0]
        
        print(ca.norm_2(self.state_init - self.state_target))
        pub_msg.linear.x = (float)(self.control[0])
        pub_msg.angular.z = (float)(self.control[1])
        self.publisher_.publish(pub_msg)




def main(args=None):
    rclpy.init(args=args)

    mpc_node_test = mpc_class()

    rclpy.spin(mpc_node_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mpc_node_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()