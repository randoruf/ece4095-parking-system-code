# Random Tree
# This is a random tree without the '_nearest_neighbour'
# instead it returns a random node in the tree.
# _rand_node()...

import numpy as np
import matplotlib.pyplot as plt


class Node:
    def __init__(self, x: float, y: float):
        """
        we only need the parent node to build the tree.
        But we need adjacent list if we want to build a graph.
        """
        self.x = x
        self.y = y
        self.parent = None


class RandomTree:
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

    def _rand_node(self, q_rand: Node) -> Node:
        """
        return a random node in the tree.
        """
        return self.node_list[np.random.randint(0, len(self.node_list))]

    def _new_conf(self, q_near: Node, q_rand: Node) -> Node:
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

    def buildRandomTree(self):
        """
        See how to build the RRT data structure....
        http://lavalle.pl/rrt/about.html
        """
        for i in range(self.num_iter):
            q_rand = self._rand_conf()
            q_near = self._rand_node(q_rand)
            q_new = self._new_conf(q_near, q_rand)
            # each node in RRT must has it parent node!
            q_new.parent = q_near
            self.node_list.append(q_new)

    def draw_RT_node(self):
        for i in range(len(self.node_list)):
            n = self.node_list[i]
            plt.plot(n.x, n.y, 'k.')

    def draw_RT_edge(self):
        """
        to demonstrate why 'parent node' information is important.
        """
        self.draw_RT_node()
        for i in range(len(self.node_list)):
            n = self.node_list[i]
            plt.plot([n.x, n.parent.x], [n.y, n.parent.y], 'g-')


q_start = Node(0, 0)
num_tree_node = 500
step_size = 5
t = RandomTree(q_start, num_tree_node, step_size, -50, 50, -50, 50)
t.buildRandomTree()

t.draw_RT_node()
plt.show()

t.draw_RT_edge()
plt.show()

