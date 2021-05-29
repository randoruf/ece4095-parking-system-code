import matplotlib.pyplot as plt
import numpy as np
from vehicle import Car
import sys
import os
from skimage import color
import skimage.transform
from vehicle import VehicleParam


class ParkingSpace:
    # see the 'delta' parameter in SLAM package in ROS http://wiki.ros.org/gmapping
    resolution = 0.05       # Resolution of the map (in metres per occupancy grid block)
    max_num_cars = 100      # the number of cars that our simulator supports.

    def __init__(self, parking_lot_name: str):
        self._parking_space_x = []
        self._parking_space_y = []
        self._parking_space_theta = []
        self._parking_lot_name = parking_lot_name
        # costmap resolution, https://zhuanlan.zhihu.com/p/143202720
        # self.resolution = 0.05      # each cell has the size of 0.05 [m],

    def append_new_parking_space(self, x: float, y: float, theta: float):
        self._parking_space_x.append(x)
        self._parking_space_y.append(y)
        self._parking_space_theta.append(theta)

    def __len__(self):
        return len(self._parking_space_x)

    def output2file(self):
        filename = os.path.splitext(self._parking_lot_name)[0] + '_parking_space.txt'
        fp = open("../benchmark_problem/" + filename, 'w')
        # the parking lot image
        fp.write("benchmark_problem/" + self._parking_lot_name + "\n")
        # the resolution of occupancy grid map
        fp.write(str(self.resolution) + "\n")
        # the maximum number of cars that support
        fp.write(str(self.max_num_cars) + "\n")
        # the dimension of the car
        # some constants of the car (https://au.mathworks.com/help/driving/ref/sedan.html)
        fp.write(str(VehicleParam.car_inflation_radius) + "\n")
        fp.write(str(VehicleParam.car_width) + "\n")
        fp.write(str(VehicleParam.car_length) + "\n")
        fp.write(str(VehicleParam.car_rf_length) + "\n")
        fp.write(str(VehicleParam.car_rb_length) + "\n")
        fp.write(str(VehicleParam.car_wheel_base) + "\n")
        fp.write(str(VehicleParam.car_max_speed) + "\n")
        fp.write(str(VehicleParam.car_max_steer_angle) + "\n")
        fp.write(str(VehicleParam.car_max_steer_rate) + "\n")
        #
        for i in range(len(self)):
            line = str(self._parking_space_x[i]) + " " + str(self._parking_space_y[i]) + " " + \
                   str(self._parking_space_theta[i]) + "\n"
            print(line)
            fp.write(line)
        fp.close()


class Cursor:
    def __init__(self, axis, icar: Car, ips: ParkingSpace):
        self.axis = axis
        self.car = icar
        self.park_space = ips

        # text location in axes coordinate
        self.txt = ax.text(0.0, 0.0, '', transform=ax.transAxes)

    def mouse_move(self, event):
        if not event.inaxes:
            return

        x, y = event.xdata, event.ydata
        # update the line positions
        _, _, theta, _ = self.car.get_state()
        self.car.update_state(x, y, theta, 0)
        self.car.vehicle_inflation_example_update()

        self.txt.set_text("x=%1.2f, y=%1.2f, theta=%1.1f" % (x, y, np.rad2deg(theta)))
        self.axis.figure.canvas.draw()

    def on_press(self, event):
        sys.stdout.flush()
        if event.key == "left" or event.key == "up":
            x, y, theta, _ = self.car.get_state()
            if event.key == "up":
                d = 0.01
            else:
                d = 0.05
            theta += d
            if theta > np.deg2rad(180):
                theta = np.deg2rad(-180+1)
            # print(np.rad2deg(theta))
            self.car.update_state(x, y, theta, 0)
            self.car.vehicle_inflation_example_update()
            self.axis.figure.canvas.draw()
            self.txt.set_text("x=%1.2f, y=%1.2f, theta=%1.1f" % (x, y, np.rad2deg(theta)))
        elif event.key == "right" or event.key == "down":
            x, y, theta, _ = self.car.get_state()
            if event.key == "down":
                d = 0.01
            else:
                d = 0.05
            theta -= d
            if theta <= np.deg2rad(-180):
                theta = np.deg2rad(180)
            # print(np.rad2deg(theta))
            self.car.update_state(x, y, theta, 0)
            # collision checking example
            self.car.vehicle_inflation_example_update()
            self.axis.figure.canvas.draw()
            self.txt.set_text("x=%1.2f, y=%1.2f, theta=%1.1f" % (x, y, np.rad2deg(theta)))

    def on_click(self, event):
        x, y, theta, _ = self.car.get_state()
        self.park_space.append_new_parking_space(x, y, theta)
        new_car = Car(self.axis, "gray")
        new_car.update_state(x, y, theta, 0)


if __name__ == "__main__":
    fig, ax = plt.subplots(subplot_kw={"aspect": "equal"})
    my_car = Car(ax, "black")

    # ignore the alpha value
    parking_lot_map_filename = "parking_lot01.png"
    parking_lot = plt.imread("../benchmark_problem/" + parking_lot_map_filename)[:, :, :3]
    parking_lot = color.rgb2gray(parking_lot)
    parking_spaces = ParkingSpace(parking_lot_map_filename)
    x_max = parking_lot.shape[1]*ParkingSpace.resolution
    y_max = parking_lot.shape[0]*ParkingSpace.resolution
    ax.set_xlim(0, x_max)
    ax.set_ylim(0, y_max)
    # How to plot over image.... https://stackoverflow.com/questions/34458251/plot-over-an-image-background-in-python
    ax.imshow(parking_lot, extent=[0, x_max, 0, y_max], origin="lower", cmap='gray')

    cursor = Cursor(ax, my_car, parking_spaces)
    fig.canvas.mpl_connect("motion_notify_event", cursor.mouse_move)
    fig.canvas.mpl_connect("key_press_event", cursor.on_press)
    fig.canvas.mpl_connect("button_press_event", cursor.on_click)

    # figure windows
    plt.show()

    # some clean up after closing the window...
    parking_spaces.output2file()
