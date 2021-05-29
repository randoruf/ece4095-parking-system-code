import numpy as np
import math
import matplotlib.pyplot as plt
from numpy import pi as PI
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import time
from typing import Tuple

# --------------------------------------------------------
# For type annotation, but no other things
from matplotlib.axes import Axes


# --------------------------------------------------------


class VehicleParam:
    car_inflation_radius = 1.7
    car_width = 2.20  # [m] width of vehicle
    car_length = 4.82
    car_rf_length = 3.70  # [m] distance from rear axle to front end of vehicle
    car_rb_length = 1.12  # [m] distance from rear axle to back end of vehicle
    car_wheel_base = 2.6  # [m] distance from rear axle to front axle
    # -------------------------------------------------------------------
    # the maximum speed of the vehicle [m/s]
    #   https://au.mathworks.com/help/driving/ug/automated-parking-valet.html
    car_max_speed = 5.0
    # -------------------------------------------------------------------
    # the maximum steering angle [radian]
    car_max_steer_angle = PI/6
    # -------------------------------------------------------------------
    # the rate of changing steering angle [radian/s], I assume 6 degree/s
    #   https://wenda.autohome.com.cn/topic/detail/152699
    car_max_steer_rate = 0.1


class Car:
    def __init__(self, axis: Axes, car_color="gray"):
        self.axis = axis
        self.car_color = car_color
        # draw the car shape
        self._initial_car_shape()
        self._initial_arrow_shape()
        self._initial_wheel_shape()
        # initial configuration...
        self.x = 0
        self.y = 0
        self.theta = 0
        self.steer_angle = 0
        self.update_state(0, 0, 0, 0)
        # collision checking example
        self.vehicle_front_circle_patch = None
        self.vehicle_rear_circle_patch = None
        self.vehicle_inflation_example_initialization(car_color)

    def _initial_car_shape(self):
        # some constants of the car (https://au.mathworks.com/help/driving/ref/sedan.html)
        self.width = VehicleParam.car_width
        self.length = VehicleParam.car_length
        self.rf_length = VehicleParam.car_rf_length
        self.rb_length = VehicleParam.car_rb_length
        self.wheel_base = VehicleParam.car_wheel_base

        self.tyre_width = 0.2
        self.tyre_length = 1

        # draw the rectangle shape of the car assume the base coordinate frame is (0, 0, 0)
        self.car_base = np.array([[-self.rb_length, -self.rb_length, self.rf_length, self.rf_length, -self.rb_length],
                                  [self.width / 2, -self.width / 2, -self.width / 2, self.width / 2, self.width / 2]])
        # plot the car shop in the given axis
        self.car_artist, = self.axis.plot(self.car_base[0, :], self.car_base[1, :], color=self.car_color)

    def get_car_dimension(self):
        return self.width, self.length, self.rf_length, self.rb_length, self.wheel_base

    def _initial_arrow_shape(self):
        x = 0
        y = 0
        theta = 0

        L = 2

        angle = np.deg2rad(30)
        d = 0.3 * L
        w = 2

        x_start = x
        y_start = y
        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + (PI - angle)
        theta_hat_R = theta + (PI + angle)

        x_hat_start = x_end
        x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

        y_hat_start = y_end
        y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

        self.arrow_start_base = np.array([[x_start, x_end], [y_start, y_end]])
        self.arrow_hat_left_base = np.array([[x_hat_start, x_hat_end_L], [y_hat_start, y_hat_end_L]])
        self.arrow_hat_right_base = np.array([[x_hat_start, x_hat_end_R], [y_hat_start, y_hat_end_R]])

        self.arrow_start_artist, = self.axis.plot(self.arrow_start_base[0, :], self.arrow_start_base[1, :],
                                                  color=self.car_color, linewidth=w)
        self.arrow_hat_left_artist, = self.axis.plot(self.arrow_hat_left_base[0, :], self.arrow_hat_left_base[1, :],
                                                     color=self.car_color, linewidth=w)
        self.arrow_hat_right_artist, = self.axis.plot(self.arrow_hat_right_base[0, :], self.arrow_hat_right_base[1, :],
                                                      color=self.car_color, linewidth=w)

    def _initial_wheel_shape(self):
        self.bl_wheel_base = np.array([[-self.tyre_length / 2, self.tyre_length / 2],
                                       [self.width / 2.8, self.width / 2.8]])
        self.br_wheel_base = np.array([[-self.tyre_length / 2, self.tyre_length / 2],
                                       [-self.width / 2.8, -self.width / 2.8]])

        self.fr_wheel_base = np.array([[-self.tyre_length / 2, self.tyre_length / 2],
                                       [0, 0]])
        self.fl_wheel_base = np.array([[-self.tyre_length / 2, self.tyre_length / 2],
                                       [0, 0]])

        self.fl_wheel_offset = np.array([[self.wheel_base, self.wheel_base],
                                         [self.width / 2.8, self.width / 2.8]])

        self.fr_wheel_offset = np.array([[self.wheel_base, self.wheel_base],
                                         [-self.width / 2.8, -self.width / 2.8]])

        self.bl_wheel_artist, = self.axis.plot(self.bl_wheel_base[0, :], self.bl_wheel_base[1, :],
                                               color=self.car_color, linewidth=3)
        self.br_wheel_artist, = self.axis.plot(self.br_wheel_base[0, :], self.br_wheel_base[1, :],
                                               color=self.car_color, linewidth=3)
        self.fl_wheel_artist, = self.axis.plot([], [], color=self.car_color, linewidth=3)
        self.fr_wheel_artist, = self.axis.plot([], [], color=self.car_color, linewidth=3)

    def get_state(self):
        return self.x, self.y, self.theta, self.steer_angle

    def update_state(self, x: float, y: float, theta: float, steer_angle: float):
        """
        Update the configuration/state of the vehicle with the state input (x, y, theta).
        (Note that this function may not respect the differential constraints).
        :param x: the location in x coordinate
        :param y: the location in y coordinate
        :param theta: the orientation of the vehicle
        :param steer_angle: the steering angle.
        """
        # The steering angle definition is different from knowledge in Primary School!
        #   http://street.umn.edu/VehControl/javahelp/HTML/Definition_of_Vehicle_Heading_and_Steeing_Angle.htm
        # assert -PI <= theta <= PI, "positive heading is 180 degree, and negative heading is -180 degree (radian)."
        # assert x >= 0 and y >= 0, "Only using the first quadrant"
        # assert -VehicleParam.car_max_steer_angle <= steer_angle <= VehicleParam.car_max_steer_angle,
        # "Check steering angle"

        self.x = x
        self.y = y
        self.theta = theta
        self.steer_angle = steer_angle

        # directly update the object in the figure!
        # the base coordinate of the car
        car = self.car_base.copy()
        # update the car location using the rotation matrix, https://en.wikipedia.org/wiki/Rotation_matrix
        rot_mat = np.array([[math.cos(theta), -math.sin(theta)],
                            [math.sin(theta), math.cos(theta)]])
        car = np.dot(rot_mat, car)
        # translation matrix
        trans_mat = np.array([[x], [y]])
        car += trans_mat
        # update the car location/orientation
        self.car_artist.set_data(car[0, :], car[1, :])

        # the base coordinate of the arrow components
        arrow_middle = self.arrow_start_base.copy()
        arrow_hat_left = self.arrow_hat_left_base.copy()
        arrow_hat_right = self.arrow_hat_right_base.copy()
        # update the car location using the rotation matrix,
        arrow_middle = np.dot(rot_mat, arrow_middle)
        arrow_hat_left = np.dot(rot_mat, arrow_hat_left)
        arrow_hat_right = np.dot(rot_mat, arrow_hat_right)
        # translation matrix
        arrow_middle += trans_mat
        arrow_hat_right += trans_mat
        arrow_hat_left += trans_mat
        # update the arrow location/orientation
        self.arrow_start_artist.set_data(arrow_middle[0, :], arrow_middle[1, :])
        self.arrow_hat_left_artist.set_data(arrow_hat_left[0, :], arrow_hat_left[1, :])
        self.arrow_hat_right_artist.set_data(arrow_hat_right[0, :], arrow_hat_right[1, :])

        # update the back wheel
        back_left_wheel = np.dot(rot_mat, self.bl_wheel_base.copy()) + trans_mat
        back_right_wheel = np.dot(rot_mat, self.br_wheel_base.copy()) + trans_mat
        self.bl_wheel_artist.set_data(back_left_wheel[0, :], back_left_wheel[1, :])
        self.br_wheel_artist.set_data(back_right_wheel[0, :], back_right_wheel[1, :])

        # update the front wheel
        rot_mat2 = np.array([[math.cos(steer_angle), -math.sin(steer_angle)],
                             [math.sin(steer_angle), math.cos(steer_angle)]])
        front_left_wheel = np.dot(rot_mat2, self.fl_wheel_base.copy())
        front_right_wheel = front_left_wheel.copy()
        front_left_wheel += self.fl_wheel_offset
        front_left_wheel = np.dot(rot_mat, front_left_wheel)

        front_left_wheel += trans_mat
        self.fl_wheel_artist.set_data(front_left_wheel[0, :], front_left_wheel[1, :])

        front_right_wheel += self.fr_wheel_offset
        front_right_wheel = np.dot(rot_mat, front_right_wheel)
        front_right_wheel += trans_mat
        self.fr_wheel_artist.set_data(front_right_wheel[0, :], front_right_wheel[1, :])

    def vehicle_inflation_example_initialization(self, car_color):
        """
            This function is useless. It is an example for the paper
        Fast collision checking for intelligent vehicle motion planning (https://ieeexplore.ieee.org/document/5547976)
        """
        # add the circle to the front axle
        self.vehicle_rear_circle_patch = patches.Circle((-1, -1), radius=VehicleParam.car_inflation_radius,
                                                        fc=car_color, alpha=0.5)
        self.axis.add_patch(self.vehicle_rear_circle_patch)
        # add the circle to the rear axle
        x_f = -1 + self.wheel_base * math.cos(self.theta)
        y_f = -1 + self.wheel_base * math.sin(self.theta)
        self.vehicle_front_circle_patch = patches.Circle((x_f, y_f), radius=VehicleParam.car_inflation_radius,
                                                         fc=car_color, alpha=0.5)
        self.axis.add_patch(self.vehicle_front_circle_patch)

    def vehicle_inflation_example_update(self):
        """
            This function is useless. It is an example for the paper
        Fast collision checking for intelligent vehicle motion planning (https://ieeexplore.ieee.org/document/5547976)
        """
        # ------------------------------------------------
        # the circle at the front axle
        x_r, y_r, _, _ = self.get_state()
        self.vehicle_rear_circle_patch.center = (x_r, y_r)

        # the circle at the rear axle (translation to the direction of theta)
        #   https://github.com/randoruf/MotionPlanning/blob/master/HybridAstarPlanner/hybrid_astar.py
        x_f = x_r + self.wheel_base * math.cos(self.theta)
        y_f = y_r + self.wheel_base * math.sin(self.theta)
        self.vehicle_front_circle_patch.center = (x_f, y_f)

    def vehicle_inflation_example_disable(self):
        """
            hide the the inflation circles
        """
        self.vehicle_rear_circle_patch.set_alpha(0)
        self.vehicle_front_circle_patch.set_alpha(0)


def animation_testing():
    fig, axis = plt.subplots(subplot_kw={'aspect': 'equal'})
    axis.set_xlim(-2, 30)
    axis.set_ylim(-2, 30)
    axis.grid(True)
    c = Car(axis, "black")

    N = 500
    r = np.linspace(-180, 180, N, endpoint=True)
    d = np.linspace(0, 30 - 5, N, endpoint=True)
    steer = np.linspace(-30, 30, N, endpoint=True)

    def update(i: int):
        """
        :param i: the frame number
        """
        c.update_state(d[i], d[i], np.deg2rad(r[i]), np.deg2rad(steer[i]))

    # interval: the delay between frames in milliseconds.
    # frames: how many frames/iterations you want to show ?
    # repeat: stop once finishing show all frames.
    animation = FuncAnimation(fig, update, interval=10, frames=N, repeat=False)
    # animation.save("movie.mp4")
    plt.show()


if __name__ == "__main__":
    animation_testing()
