# dynamic_window_approach.py: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py

from turtlesim.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import math
import numpy as np

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        self.obs_x = 5.0
        self.obs_y = 5.0
        self.ob = None
        self.declare_parameter('num_target', 3)
        self.num_target = self.get_parameter('num_target').value
        self.declare_parameter('pick_target', 325)
        self.pick_target = self.get_parameter('pick_target').value
        self.declare_parameter('max_speed', 2.0)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('min_speed', -0.5)
        self.min_speed = self.get_parameter('min_speed').value
        self.declare_parameter('max_yaw_rate', 0.69777)
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.declare_parameter('max_accel', 0.2)
        self.max_accel = self.get_parameter('max_accel').value
        self.declare_parameter('max_delta_yaw_rate', 0.8)
        self.max_delta_yaw_rate = self.get_parameter('max_delta_yaw_rate').value
        self.declare_parameter('v_resolution', 0.01)
        self.v_resolution = self.get_parameter('v_resolution').value
        self.declare_parameter('yaw_rate_resolution', 0.0017444)
        self.yaw_rate_resolution = self.get_parameter('yaw_rate_resolution').value
        self.declare_parameter('dt', 0.2)
        self.dt = self.get_parameter('dt').value
        self.declare_parameter('predict_time', 4.0)
        self.predict_time = self.get_parameter('predict_time').value
        self.declare_parameter('to_goal_cost_gain', 2.0)
        self.to_goal_cost_gain = self.get_parameter('to_goal_cost_gain').value
        self.declare_parameter('speed_cost_gain', 1.0)
        self.speed_cost_gain = self.get_parameter('speed_cost_gain').value
        self.declare_parameter('obstacle_cost_gain', 0.5)
        self.obstacle_cost_gain = self.get_parameter('obstacle_cost_gain').value
        self.declare_parameter('robot_stuck_flag_cons', 0.1)
        self.robot_stuck_flag_cons = self.get_parameter('robot_stuck_flag_cons').value
        self.declare_parameter('robot_type', 0)
        self.robot_type = self.get_parameter('robot_type').value

        self.goal_tolerance = 0.5

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        self.robot_radius = 1.0  # [m] for collision check (circle)
        self.robot_width = 0.5  # [m] for collision check (rectangle)
        self.robot_length = 1.2  # [m] for collision check (rectangle)

        self.x = np.array([0, 0, 0, 0, 0])
        self.obs_sub_finish = False
        self.agent_sub_finish = False

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.total_target = int(math.pow(self.num_target, 2))
        self.bin_pick_target = bin(self.pick_target)[2:]
        while len(self.bin_pick_target) < self.total_target:
            self.bin_pick_target = '0' + self.bin_pick_target
        # print(self.bin_pick_target)

        self.selected_target_list = list()
        for num, i in enumerate(self.bin_pick_target[::-1]):
            if i == '0':
                pass
            elif i == '1':
                self.selected_target_list.append(num)
        # print(self.selected_target_list)
        if len(self.selected_target_list) == 0:
            raise TypeError("None of the targets are selected")

        self.target = np.empty((0, 2))
        for selected_target in self.selected_target_list:
            # total width of turtlesim's place is 11
            target_x = (int(selected_target % self.num_target) + 1) * (11 / (self.num_target + 1))
            target_y = (int(selected_target // self.num_target) + 1) * (11 / (self.num_target + 1))
            self.target = np.vstack((self.target, np.array([target_x, target_y])))

        self.obs_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.set_obstacle_pose,
            QOS_RKL10V
            )

        self.agent_subscriber = self.create_subscription(
            Pose,
            '/agent/pose',
            self.set_agent_pose,
            QOS_RKL10V)

        self.twist_publisher = self.create_publisher(
            Twist,
            '/agent/cmd_vel',
            QOS_RKL10V)

        self.timer = self.create_timer(self.dt, self.publish_twist_msg)

    def publish_twist_msg(self):
        self.lin_vec = Vector3()
        self.ang_vec = Vector3()
        self.twist = Twist()

        self.lin_vec.x, self.lin_vec.y, self.lin_vec.z = float(self.x[3]), 0.0, 0.0
        self.ang_vec.x, self.ang_vec.y, self.ang_vec.z = 0.0, 0.0, float(self.x[4])
        self.twist.linear = self.lin_vec
        self.twist.angular = self.ang_vec

        self.twist_publisher.publish(self.twist)
        #self.get_logger().info(f'Published Linear Velocity: {float(self.x[3])}')
        #self.get_logger().info(f'Published Angular Velocity: {float(self.x[4])}')

    def set_obstacle_pose(self, msg):
        self.obs_x = msg.x
        self.obs_y = msg.y
        self.ob = np.array([[self.obs_x, self.obs_y]])
        self.obs_sub_finish = True

    def set_agent_pose(self, msg):
        # state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.x = np.array([msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity])
        self.agent_sub_finish = True

    def update_parameter(self, params):
        for param in params:
            if param.name == 'num_target' and param.type_ == Parameter.Type.INTEGER:
                self.num_target = param.value
            elif param.name == 'pick_target' and param.type_ == Parameter.Type.INTEGER:
                self.pick_target = param.value
            elif param.name == 'max_speed':
                self.max_speed = param.value
            elif param.name == 'min_speed':
                self.min_speed = param.value
            elif param.name == 'max_yaw_rate':
                self.max_yaw_rate = param.value
            elif param.name == 'max_accel':
                self.max_accel = param.value
            elif param.name == 'max_delta_yaw_rate':
                self.max_delta_yaw_rate = param.value
            elif param.name == 'v_resolution':
                self.v_resolution = param.value
            elif param.name == 'yaw_rate_resolution':
                self.yaw_rate_resolution = param.value
            elif param.name == 'dt':
                self.dt = param.value
            elif param.name == 'predict_time':
                self.predict_time = param.value
            elif param.name == 'to_goal_cost_gain':
                self.to_goal_cost_gain = param.value
            elif param.name == 'speed_cost_gain':
                self.speed_cost_gain = param.value
            elif param.name == 'obstacle_cost_gain':
                self.obstacle_cost_gain = param.value
            elif param.name == 'robot_stuck_flag_cons':
                self.robot_stuck_flag_cons = param.value
            elif param.name == 'robot_type' and param.type_ == Parameter.Type.INTEGER:
                self.robot_type = param.value
        return SetParametersResult(successful=True)

    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw


    def predict_trajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self.motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory


    def calc_control_and_trajectory(self, x, dw, goal, ob):
        """
        calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -self.max_delta_yaw_rate
        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if self.robot_type == 1:
            yaw = trajectory[:, 2]
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            rot = np.transpose(rot, [2, 0, 1])
            local_ob = ob[:, None] - trajectory[:, 0:2]
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            local_ob = np.array([local_ob @ x for x in rot])
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            upper_check = local_ob[:, 0] <= self.robot_length / 2
            right_check = local_ob[:, 1] <= self.robot_width / 2
            bottom_check = local_ob[:, 0] >= -self.robot_length / 2
            left_check = local_ob[:, 1] >= -self.robot_width / 2
            if (np.logical_and(np.logical_and(upper_check, right_check),
                            np.logical_and(bottom_check, left_check))).any():
                return float("Inf")
        elif self.robot_type == 0:
            if np.array(r <= self.robot_radius).any():
                return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def dwa_control(self, x, goal, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, ob)

        self.x = trajectory[1]
        self.publish_twist_msg()

        return u, trajectory


def main(args=None):
    rclpy.init(args=args)
    try:
        dwa_planner = DWAPlanner()
        i = 0
        try:
            while True:
            #for _ in range(1):
                rclpy.spin_once(dwa_planner)
                goal = dwa_planner.target[i]
                ob = dwa_planner.ob
                if dwa_planner.obs_sub_finish == True and dwa_planner.agent_sub_finish == True:
                    #print("ob:", ob)
                    #print("x: ", dwa_planner.x)
                    print("goal: ",  goal)
                    dwa_planner.dwa_control(dwa_planner.x, goal, ob)
                if np.sqrt(np.sum(np.square(dwa_planner.x[:2] - goal))) <= dwa_planner.goal_tolerance:
                    print("Goal arrived", dwa_planner.x[:2])
                    i += 1
                    if i == dwa_planner.target.shape[0]:
                        i = 0
                    goal = dwa_planner.target[i]

        except KeyboardInterrupt:
            dwa_planner.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            dwa_planner.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
