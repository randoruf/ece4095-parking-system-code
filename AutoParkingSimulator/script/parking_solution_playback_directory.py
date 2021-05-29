import os


if __name__ == "__main__":
    python_header = "python parking_solution_playback.py ../benchmark_problem/parking_lot01.png "
    data_directory = "../experimental_data/five_agent/"

    # os.system("2020_05_20_greedy_euclidean_goal_agent1_instance1.txt")
    for file in os.listdir(data_directory):
        os.system(python_header + data_directory + file)

