# The basic implementation of RRT
#   - We only need the 'parent node' information in a node
#      (to search the goal, or draw edges)
#
# Some reference about RRT
#   - The author of RRT: http://lavalle.pl/rrt/about.html


import numpy as np
import matplotlib.pyplot as plt
from vehicle import Car
from matplotlib.animation import FuncAnimation, PillowWriter


class Node:
    def __init__(self, x: float, y: float):
        """
        we only need the parent node to build the tree.
        But we need adjacent list if we want to build a graph.
        """
        self.x = x
        self.y = y
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

    def _rand_conf(self) -> Node:
        """
        sample a random configuration from the state space
        """
        x_rand = np.random.uniform(self.x_min, self.x_max, 1)
        y_rand = np.random.uniform(self.y_min, self.y_max, 1)
        return Node(x_rand[0], y_rand[0])

    def _nearest_neighbour(self, q_rand: Node) -> Node:
        """
        sequential search to find the nearest neighbour.
        """
        min_d = float("inf")
        min_d_index = -1

        for i in range(0, len(self.node_list)):
            dx = q_rand.x - self.node_list[i].x
            dy = q_rand.y - self.node_list[i].y
            d = np.sqrt(dx*dx + dy*dy)
            if d < min_d:
                min_d = d
                min_d_index = i

        return self.node_list[min_d_index]

    def _steer(self, q_near: Node, q_rand: Node) -> Node:
        """
        pick a point between the random configuration and its nearest neighbour...
        """
        dx = q_rand.x - q_near.x
        dy = q_rand.y - q_near.y
        d = np.sqrt(dx * dx + dy * dy)

        # to restrict/boost the growth of the tree. Let it be BFS
        #   - normalize the distance between q_near and q_rand by (1/d * q_rand.x)
        #   - then move forward
        nx = q_near.x + (self.delta_q / d) * dx
        ny = q_near.y + (self.delta_q / d) * dy

        return Node(nx, ny)

    def buildRRT(self):
        """
        See how to build the RRT data structure....
        http://lavalle.pl/rrt/about.html
        """
        for i in range(self.num_iter):
            q_rand = self._rand_conf()
            q_near = self._nearest_neighbour(q_rand)
            q_new = self._steer(q_near, q_rand)
            # each node in RRT must has it parent node!
            q_new.parent = q_near
            self.node_list.append(q_new)

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
    bounding_box = 50
    q_start = Node(bounding_box//2, bounding_box//2)
    num_tree_node = 100
    step_size = 5
    t = RRTnoGoal(q_start, num_tree_node, step_size, x_min=0, x_max=bounding_box, y_min=0, y_max=bounding_box)
    ax.set_xlim(-2, bounding_box)
    ax.set_ylim(-2, bounding_box)
    ax.grid(True)
    t.buildRRT()

    # draw the car
    tmp_car = Car(ax, "blue")
    tmp_car.update_state(q_start.x, q_start.y, np.deg2rad(90), 0)
    tmp_car.vehicle_inflation_example_disable()

    # draw the edge
    def update(i: int):
        if i < num_tree_node:
            t.draw_RRT_edge(i, ax)

    animation = FuncAnimation(fig, update, interval=10, frames=(num_tree_node + 20), repeat=False)

    # save as gif
    writer = PillowWriter(fps=25)
    animation.save("demo_sine.gif", writer=writer)
    # plt.show()
