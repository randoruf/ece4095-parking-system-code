# The basic implementation of RRT
#   - We only need the 'parent node' information in a node
#      (to search the goal, or draw edges)
# ----------------------------------------------------------------------------------------------------------------------
# RRT official website
#   - http://lavalle.pl/rrt/about.html
# Master thesis: RRT-based path planning and model predictive control for an autonomous race car by Maxim Yastremsky
#   - https://www.youtube.com/watch?v=eOevF5jFSoc
# Kinodynamic RRTs with Fixed Time Step and Best-Input Extension Are Not Probabilistically Complete
# by Tobias Kunz and Mike Stilman
#   - https://link.springer.com/chapter/10.1007/978-3-319-16595-0_14
# In this paper, the authors prove that random input and fixed time step is probabilistically complete.
# To line up with RRT, you need to use this method.
# ----------------------------------------------------------------------------------------------------------------------
# But I want to use variable time step and random control input
# ----------------------------------------------------------------------------------------------------------------------

import numpy as np
import matplotlib.pyplot as plt
from vehicle import Car, VehicleParam
from matplotlib.animation import FuncAnimation, PillowWriter


class Model2DRigidCarSmooth:
    def __init__(self, car_length: float, x_max: float, y_max: float):
        # The state (x, y, theta, steer_angle)
        # The input (v, phi)
        # self.state_dimension = 4
        # self.input_dimension = 2
        self.speed = 1
        self.car_length = car_length
        self.h = 0.01
        # the rotation of the configuration space
        self.theta_lower_bound = -np.pi
        self.theta_upper_bound = np.pi
        # the bounding box of the RRT sampling space
        self.x_upper_bound = x_max
        self.y_upper_bound = y_max

        # maximum steering angle (30 degree)
        self.steer_angle_lower_bound = -np.pi/6
        self.steer_angle_upper_bound = np.pi/6

    def euler_integrate(self, x: np.ndarray, u: np.ndarray, dt: float):
        """
        x is the state (x, y, theta, steer_angle)
        u is the control input (v, steer), note that u[0] can only be [-1, 1].
        h is the simulation step size in ODE integration
        """
        return x + dt * self.state_transition_equation(x, u)

    def state_transition_equation(self, x: np.ndarray, u: np.ndarray):
        """
            x is the state (x, y, theta, steer_angle)
            u is the control input (v, steer), note that u[0] can only be [-1, 1].
        """

        x_loc = self.speed * u[0] * np.cos(x[2])
        y_loc = self.speed * u[0] * np.sin(x[2])
        theta_loc = np.tan(u[1]) / self.car_length
        return np.array([x_loc, y_loc, theta_loc])

    def integrate(self, x: np.ndarray, u: np.ndarray, interval: float):
        """
        x is the state (x, y, theta, steer_angle)
        u is the control input (v, steer), note that u[0] can only be [-1, 1].
        h is the simulation step size in ODE integration
        """


    def valid_state(self, x: np.ndarray) -> bool:
        """
        the bounding radius (only sample inside a big circle)
        the orientation and steering angle
        """
        return 0 <= x[0] <= self.x_upper_bound and 0 <= x[1] <= self.y_upper_bound and -np.pi <= x[2] < np.pi

    @staticmethod
    def distance_metric(x1: np.ndarray, x2: np.ndarray):
        dx = x1[0] - x2[0]
        dy = x1[1] - x2[1]
        dtheta = min(abs(x1[2] - x2[2]), 2 * np.pi - abs(x1[2] - x2[2]))
        weighted_dtheta = (40/np.pi) * dtheta

        return np.sqrt(
            dx * dx +
            dy * dy +
            weighted_dtheta * weighted_dtheta
        )


class Node:
    def __init__(self, x: float, y: float, theta: float):
        """
        we only need the parent node to build the tree.
        But we need adjacent list if we want to build a graph.
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = None


class RRTnoGoal:
    """
    create the RRT tree data structure (without goal).
    http://lavalle.pl/rrt/about.html
    """
    def __init__(self, q_int: Node, num_iter: int, delta_q: float, x_min: int, x_max: int, y_min: int, y_max: int):
        """
        Input: initialize the tree.
        :param q_int: the initial configuration
        :param num_iter: the number of iteration/vertices in RRT
        :param delta_q: the incremental distance
        """
        self.q_int = q_int
        self.q_int.parent = q_int
        self.num_iter = num_iter
        self.delta_q = delta_q
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        # the maintain all nodes in the tree.
        self.node_list = [self.q_int]
        # the car model
        self.car_model = Model2DRigidCarSmooth(VehicleParam.car_wheel_base, x_max, y_max)
        # the random sample was used...s
        self.rand_node_list = [self.q_int]

    def _rand_conf(self) -> Node:
        """
        sample a random configuration from the state space
        """
        x_rand = np.random.uniform(self.x_min, self.x_max, 1)
        y_rand = np.random.uniform(self.y_min, self.y_max, 1)
        theta_rand = np.random.uniform(-np.pi, np.pi)  # [low, high)
        return Node(x_rand[0], y_rand[0], theta_rand)

    def _nearest_neighbour(self, q_rand: Node) -> Node:
        """
        sequential search to find the nearest neighbour.
        """
        min_d = float("inf")
        min_d_index = -1

        for i in range(0, len(self.node_list)):
            x_rand = np.array([q_rand.x, q_rand.y, q_rand.theta])
            x_tmp = np.array([self.node_list[i].x, self.node_list[i].y,
                              self.node_list[i].theta]
                             )
            d = self.car_model.distance_metric(x_rand, x_tmp)
            if d < min_d:
                min_d = d
                min_d_index = i

        return self.node_list[min_d_index]

    def _steer(self, q_near: Node, q_rand: Node) -> Node:
        """
        pick a point between the random configuration and its nearest neighbour...
        """
        x_near = np.array([q_near.x, q_near.y, q_near.theta])
        x_rand = np.array([q_rand.x, q_rand.y, q_rand.theta])

        d_min = float("inf")
        u_best = np.array([1, self.car_model.steer_rate])
        t_best = 1
        nx_best = self.car_model.integrate(np.array(x_near), u_best, t_best)

        # choose the best inputs (velocity and steering angle)
        for velocity_direct in [-1, 1]:                         # velocity direction
            h = np.random.uniform(low=1, high=3)
            u = np.array([velocity_direct, np.random.uniform(-np.pi/6, np.pi/6)])
            nx = self.car_model.integrate(
                np.array(x_near),
                u,
                h
            )

            #
            if self.car_model.distance_metric(x_rand, nx) < d_min:
                d_min = self.car_model.distance_metric(x_rand, nx)
                u_best = u
                nx_best = nx
                t_best = h

        # print(x_rand[0], x_rand[1], x_rand[2]/np.pi*180, x_rand[3]/np.pi*180)
        # print(nx_best[0], nx_best[1], nx_best[2]/np.pi*180, nx_best[3]/np.pi*180)
        # print("---------------------")
        return Node(nx_best[0], nx_best[1], nx_best[2])

    def buildRRT(self):
        """
        See how to build the RRT data structure....
        http://lavalle.pl/rrt/about.html
        """
        for i in range(self.num_iter):
            q_rand = self._rand_conf()
            q_near = self._nearest_neighbour(q_rand)
            q_new = self._steer(q_near, q_rand)
            x_new = np.array([q_new.x, q_new.y, q_new.theta])
            if self.car_model.valid_state(x_new):
                # each node in RRT must has it parent node!
                q_new.parent = q_near
                self.node_list.append(q_new)
                self.rand_node_list.append(q_rand)

    def draw_RRT_random_sample(self, i: int, axis):
        r = self.rand_node_list[i]
        axis.plot(r.x, r.y, 'k.')

    def draw_RRT_node(self, i: int, axis):
        n = self.node_list[i]
        axis.plot(n.x, n.y, 'g.', alpha=0.8)

    def draw_RRT_edge(self, i: int, axis):
        """
        to demonstrate why 'parent node' information is important.
        """
        self.draw_RRT_node(i, axis)
        n = self.node_list[i]
        axis.plot([n.x, n.parent.x], [n.y, n.parent.y], 'g-', alpha=0.5)


if __name__ == "__main__":
    fig, ax = plt.subplots(subplot_kw={"aspect": "equal"})
    # some constants
    bounding_box = 50
    num_tree_node = 500
    step_size = 5

    # build a RRT
    q_start = Node(bounding_box//2, bounding_box//2, np.deg2rad(90))
    t = RRTnoGoal(q_start, num_tree_node, step_size, x_min=0, x_max=bounding_box, y_min=0, y_max=bounding_box)
    ax.grid(True)
    t.buildRRT()

    # draw the car
    tmp_car = Car(ax, "blue")
    tmp_car.update_state(q_start.x, q_start.y, q_start.theta, 0)
    tmp_car.vehicle_inflation_example_disable()

    # draw the edge
    def update(i: int):
        # print(i)
        if i < len(t.node_list) and i < len(t.rand_node_list):
            t.draw_RRT_edge(i, ax)
            # t.draw_RRT_random_sample(i, ax)

    animation = FuncAnimation(fig, update, interval=10, frames=(num_tree_node + 20), repeat=False)

    # save as gif
    # writer = PillowWriter(fps=40)
    # animation.save("demo_sine.gif", writer=writer)
    plt.show()
