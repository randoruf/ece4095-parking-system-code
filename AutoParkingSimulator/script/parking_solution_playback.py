"""
1. Non-blocking matplotlib show()
    https://stackoverflow.com/questions/28269157/plotting-in-a-non-blocking-way-with-matplotlib
2.

"""

import matplotlib.patches
from vehicle import Car
import matplotlib.pyplot as plt
from parking_slot_generator import ParkingSpace
from matplotlib.animation import FuncAnimation
import sys


if __name__ == "__main__":
    parking_lot_img = None
    trajectory_txt = None
    collided_timestep_txt = None

    # called from terminal or Pycharm???
    if len(sys.argv) == 1:
        parking_lot_img = "../benchmark_problem/parking_lot01.png"
        trajectory_txt = "../experimental_data/five_agent/2020_05_20_greedy_euclidean_goal_agent5_instance1.txt"
        # collided_timestep_txt = "../experimental_data/he"
    else:
        print(sys.argv)
        assert len(sys.argv) == 4 or len(sys.argv) == 3, "incorrect number of parameters"
        parking_lot_img = sys.argv[1]
        trajectory_txt = sys.argv[2]
        if len(sys.argv) == 4:
            collided_timestep_txt = sys.argv[3]

    # the parking lot image
    fig, ax = plt.subplots(subplot_kw={"aspect": "equal"})
    parking_lot = plt.imread(parking_lot_img)[:, :]
    x_max = parking_lot.shape[1]*ParkingSpace.resolution
    y_max = parking_lot.shape[0]*ParkingSpace.resolution
    ax.set_xlim(0, x_max)
    ax.set_ylim(0, y_max)
    ax.imshow(parking_lot, extent=[0, x_max, 0, y_max], origin="lower", cmap='gray')

    # path way point files
    fp = open(trajectory_txt, "r")

    # create cars in the parking lot
    num_cars = int(fp.readline())
    if num_cars == 0:
        exit(1)

    car_lst = []
    car_goal_lst = []
    car_path_lst = []
    t_max = 0

    # read inputs...
    for _ in range(num_cars):
        # read the car color
        r_val, g_val, b_val = fp.readline().strip().split(' ')
        r_val = int(r_val)/255.0
        g_val = int(g_val)/255.0
        b_val = int(b_val)/255.0
        tmp_car = Car(ax, car_color=(r_val, g_val, b_val))
        tmp_car.vehicle_inflation_example_disable()
        car_lst.append(tmp_car)

        # read the car goal
        x_val, y_val, _ = fp.readline().strip().split(' ')
        x_val = float(x_val)
        y_val = float(y_val)
        # plot a dot in the destination...
        tmp_goal = matplotlib.patches.Circle((x_val, y_val), radius=1, color=(r_val, g_val, b_val))
        car_goal_lst.append(tmp_goal)
        ax.add_patch(tmp_goal)

        # read path solution
        path_len = fp.readline().strip()
        path_len = int(path_len)
        # choose the maximum path length
        if path_len > t_max:
            t_max = path_len
        # insert way points
        car_path_lst.append([])
        for _ in range(path_len):
            x_val, y_val, theta_val = fp.readline().strip().split(' ')
            x_val = float(x_val)
            y_val = float(y_val)
            theta_val = float(theta_val)
            car_path_lst[-1].append([x_val, y_val, theta_val])

        #
        x = [car_path_lst[-1][i][0] for i in range(len(car_path_lst[-1]))]
        y = [car_path_lst[-1][i][1] for i in range(len(car_path_lst[-1]))]
        ax.scatter(x, y, s=0.5, color=(r_val, g_val, b_val))

    # close the way point files
    fp.close()

    # collision time-step
    if collided_timestep_txt:
        collision_time_step_example = []
        with open(collided_timestep_txt, "r") as fp:
            for line in fp:
                collision_time_step_example.append(int(line.strip()))
        print(collision_time_step_example)

    # update for each scene
    def update(i: int):
        # print("-----------------\n", i)
        for k in range(num_cars):
            if i < len(car_path_lst[k]):
                car_lst[k].update_state(car_path_lst[k][i][0], car_path_lst[k][i][1], car_path_lst[k][i][2], 0)

                # if i in collision_time_step_example:
                #    plt.pause(1)
                #    print("collided!!")

    # generate the animation
    animation = FuncAnimation(fig, update, interval=10, frames=t_max+20, repeat=False)

    # -------------------------------------------------------------------------------------------------------
    # show the matlab's window....
    # print(t_max)
    animation.save('greedy_goal_example_presentation.gif', fps=10)
    # animation.save('multi_agent_replanning_animation_seq_01.mp4', fps=10)
    # plt.show()
