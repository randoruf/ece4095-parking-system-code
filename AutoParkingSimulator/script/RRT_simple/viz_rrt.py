import matplotlib.pyplot as plt
import numpy
from vehicle import Car
from parking_slot_generator import ParkingSpace
from matplotlib.animation import FuncAnimation

if __name__ == "__main__":
    fig, ax = plt.subplots(subplot_kw={"aspect": "equal"})
    parking_lot = plt.imread("../../parking_lot01.png")[:, :]
    x_max = parking_lot.shape[1]*ParkingSpace.resolution
    y_max = parking_lot.shape[0]*ParkingSpace.resolution
    ax.set_xlim(0, x_max)
    ax.set_ylim(0, y_max)
    ax.imshow(parking_lot, extent=[0, x_max, 0, y_max], origin="lower", cmap='gray')

    data = numpy.loadtxt('rrt_demo(1).txt')
    ax.plot(data[:, 0], data[:, 1], '.')
    c = Car(ax, "blue")
    # c.vehicle_inflation_example_disable()

    goalc = Car(ax, "gray")
    goalc.vehicle_inflation_example_disable()
    goalc.update_state(data[0, 0], data[0, 1], data[0, 2], 0)

    def update(i: int):
        """
        :param i: the frame number
        """
        c.update_state(data[i, 0], data[i, 1], data[i, 2], 0)
        c.vehicle_inflation_example_update()


    animation = FuncAnimation(fig, update, interval=10, frames=len(data), repeat=False)

    plt.show()
