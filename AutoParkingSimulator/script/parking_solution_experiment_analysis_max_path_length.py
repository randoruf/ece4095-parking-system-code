import os
import fnmatch
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    python_header = "python parking_solution_playback.py ../benchmark_problem/parking_lot01.png "
    data_directory = "../experimental_data/"
    num_agent = ["one_agent/", "two_agent/", "three_agent/", "four_agent/", "five_agent/"]
    goal_assignment_method = ["random_goal", "greedy_euclidean_goal", "prioritized_euclidean_goal"]

    # the number for all agents.
    max_path_length = [[], [], []]

    #
    for na in num_agent:
        # print("\n*****************************" + na + "******************************")

        for i in range(len(goal_assignment_method)):
            # the average path length....
            max_path_length[i].append([])
            # print(goal_assignment_method[i] + ':')
            for file in os.listdir(data_directory + na):
                if fnmatch.fnmatch(file, ('*'+goal_assignment_method[i]+'*')):
                    # print(file)
                    fp = open(data_directory + na + file, "r")
                    # num of agents in the parking lot
                    num_cars = int(fp.readline())
                    if num_cars == 0:
                        # success[i][-1].append(0)
                        pass
                    else:
                        # path length of each car
                        max_path_len = float('-inf')

                        for k in range(num_cars):
                            fp.readline()   # agent color
                            fp.readline()   # agent goal
                            path_len = int(fp.readline())   # agent solution length...
                            max_path_len = max(max_path_len, path_len)
                            # skip the path solution...
                            for _ in range(path_len):
                                fp.readline()
                        #
                        # print(total_path_len / num_cars)
                        # the average path length
                        max_path_length[i][-1].append(max_path_len)
                        fp.close()
        # print("...........................................................................")

    # ===============================================================================
    #
    # ===============================================================================
    fig, ax = plt.subplots()
    ax.set_xticks(np.arange(1, len(num_agent)+1))
    ax.set_ylim(0, 150)
    goal_assignment_color = ['blue', 'orangered', 'green']
    goal_assignment_line_style = ['-', '--', ':']

    for i in range(len(goal_assignment_method)):
        print("----------------------------------")
        print(goal_assignment_method[i] + ": ")

        average_average_path_length_num_agent = []
        average_average_path_length_num_agent_error_bar = []

        for j in range(len(num_agent)):
            average_average_path_length_num_agent.append(np.nanmean(max_path_length[i][j]))
            average_average_path_length_num_agent_error_bar.append(np.nanstd(max_path_length[i][j]))
            # print(average_average_path_length_num_agent[-1])

        ax.errorbar(np.arange(1, len(num_agent) + 1), average_average_path_length_num_agent,
                    linestyle=goal_assignment_line_style[i], marker='o', color=goal_assignment_color[i])

        print(average_average_path_length_num_agent_error_bar)

    ax.legend(goal_assignment_method)
    plt.xlabel('Number of agents')
    plt.ylabel('Maximum Path Length')
    plt.show()
