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
    success = [[], [], []]
    average_path_length = [[], [], []]
    maximum_path_length = [[], [], []]

    #
    for na in num_agent:
        # print("\n*****************************" + na + "******************************")

        for i in range(len(goal_assignment_method)):
            # the number of agent
            success[i].append([])

            summary_success_count = 0
            summary_fail_count = 0
            summary_average_path_len = 0
            summary_max_path = 0

            # print(goal_assignment_method[i] + ':')
            for file in os.listdir(data_directory + na):
                if fnmatch.fnmatch(file, ('*'+goal_assignment_method[i]+'*')):
                    # print(file)
                    fp = open(data_directory + na + file, "r")
                    # num of agents in the parking lot
                    num_cars = int(fp.readline())
                    if num_cars == 0:
                        success[i][-1].append(0)
                    else:
                        success[i][-1].append(1)

                        # path length of each car
                        max_path_len = float('-inf')
                        total_path_len = 0
                        for k in range(num_cars):
                            fp.readline()
                            fp.readline()
                            path_len = int(fp.readline())
                            total_path_len += path_len
                            max_path_len = max(max_path_len, path_len)
                            for _ in range(path_len):
                                fp.readline()
                        #
                        summary_average_path_len += (total_path_len / num_cars)
                        summary_max_path += max_path_len
                        # print(file, "Success", max_path_len, total_path_len / num_cars)
                        fp.close()

            # Summary
            # success[i].append(summary_success_count / (summary_success_count + summary_fail_count))
            # average_path_length[i].append(summary_average_path_len / summary_success_count)
            # maximum_path_length[i].append(summary_max_path / summary_success_count)
        # print("...........................................................................")

    # ===============================================================================
    #
    # ===============================================================================
    fig, ax = plt.subplots()
    ax.set_xticks(np.arange(1, len(num_agent)+1))
    ax.set_ylim(0, 1+0.1)

    goal_assignment_color = ['blue', 'orangered', 'green']
    goal_assignment_line_style = ['-', '--', ':']

    for i in range(len(goal_assignment_method)):
        print("----------------------------------")
        print(goal_assignment_method[i] + ": ")

        average_success_num_agent = []
        average_success_num_agent_error_bar = []

        for j in range(len(num_agent)):
            average_success_num_agent.append(np.nanmean(success[i][j]))
            average_success_num_agent_error_bar.append(np.nanstd(success[i][j]))

        print(average_success_num_agent)
        ax.errorbar(np.arange(1, len(num_agent)+1), average_success_num_agent,
                    linestyle=goal_assignment_line_style[i], marker='o', color=goal_assignment_color[i])

    ax.legend(goal_assignment_method)
    plt.xlabel('Number of agents')
    plt.ylabel('Success Rate')
    plt.show()


