from vehicle import Car
import matplotlib.pyplot as plt
from parking_slot_generator import ParkingSpace

if __name__ == "__main__":
    fig, ax = plt.subplots(subplot_kw={"aspect": "equal"})
    parking_lot_img = plt.imread("../benchmark_problem/parking_lot01.png")[:, :]
    x_max = parking_lot_img.shape[1] * ParkingSpace.resolution
    y_max = parking_lot_img.shape[0] * ParkingSpace.resolution
    ax.set_xlim(0, x_max)
    ax.set_ylim(0, y_max)
    ax.imshow(parking_lot_img, extent=[0, x_max, 0, y_max], origin="lower", cmap='gray')

    num_cars = 0
    with open("../benchmark_problem/parking_lot01_parking_space.txt", "r") as fp:
        for _ in range(12):
            next(fp)
        for line in fp:
            x, y, theta = line.strip().split(' ')
            tmp_car = Car(ax, "gray")
            tmp_car.update_state(float(x), float(y), float(theta), 0)
            tmp_car.vehicle_inflation_example_update()
            num_cars += 1

    # print("There are " + str(num_cars) + " parking space in the parking lot.")
    plt.show()
