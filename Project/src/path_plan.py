#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import random
import math
from shapely.geometry import LineString, Point
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def control():
    global path_new
    while not rospy.is_shutdown():
        pub.publish(path_new)


def optimize():
    global beg
    global path
    global optimized_path
    global obstacle_list
    optimized_path = Path()
    x = path.poses[0].pose.position.x
    y = path.poses[0].pose.position.y
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0
    optimized_path.poses.append(pose)

    if not check_obs_coor(
        beg[0], beg[1], path.poses[0].pose.position.x, path.poses[0].pose.position.y
    ):
        return 1

    current_index = 0
    while current_index < len(path.poses) - 1:

        index_updated = False

        for lookahead_index in range(len(path.poses) - 1, current_index, -1):
            if not check_obs_coor(
                path.poses[current_index].pose.position.x,
                path.poses[current_index].pose.position.y,
                path.poses[lookahead_index].pose.position.x,
                path.poses[lookahead_index].pose.position.y,
            ):
                x = path.poses[lookahead_index].pose.position.x
                y = path.poses[lookahead_index].pose.position.y
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0
                optimized_path.poses.append(pose)
                current_index = lookahead_index
                index_updated = True
                break

        if not index_updated:
            return 1

    return 1


def reverse():
    global optimized_path
    global path_new
    l = len(optimized_path.poses)
    for i in range(l):
        x = optimized_path.poses[-(i + 1)].pose.position.x
        y = optimized_path.poses[-(i + 1)].pose.position.y
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        path_new.poses.append(pose)


def add_to_open(open_, neighbor):
    for node in open_:
        if neighbor == node and neighbor.f > node.f:
            return False
    return True


def check_obs_coor(x1, y1, x2, y2):
    global obstacle_list
    for obstacle in obstacle_list:
        if (
            LineString([(x1, y1), (x2, y2)]).intersects(
                Point(obstacle[0], obstacle[1]).buffer(0.25 + 0.31 + 0.13)
            )
            == True
        ):
            return True
    return False


def check_obs(i, j):
    global nodes
    global obstacle_list
    for obstacle in obstacle_list:
        if (
            LineString(
                [(nodes[i][0], nodes[i][1]), (nodes[j][0], nodes[j][1])]
            ).intersects(Point(obstacle[0], obstacle[1]).buffer(0.25 + 0.31 + 0.13))
            == True
        ):
            return True
    return False


def callback(msg):
    global obstacle_list
    global p
    global path_new
    path_new = Path()
    obstacle_list = []
    for i in range(int(msg.header.frame_id)):
        x = msg.poses[i].pose.position.x
        y = msg.poses[i].pose.position.y
        obstacle_list.append([x, y])
    prm = PRM()
    print("Please wait while the robot analyses its surroundings!!")
    prm.construct()
    # prm.visualize_tree(g,obstacle_list)
    global path
    path = Path()
    global beg
    beg = [0, 0]
    end = [6, 6]
    print("Planning path")
    b = prm.plan(beg, end)
    if b == False:
        path = Path()
    optimize()
    reverse()
    print("Path Planning Complete")
    control()


def check_in_obs(l):
    global obstacle_list
    for obstacle in obstacle_list:
        dist = math.sqrt((obstacle[0] - l[0]) ** 2 + (obstacle[1] - l[1]) ** 2)
        if dist <= (0.25 + 0.31 + 0.13):
            return True
    return False


def sampler(sample_area):
    return (
        random.uniform(sample_area[0], sample_area[1]),
        random.uniform(sample_area[0], sample_area[1]),
    )


class Node:
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.name == other.name

    def __lt__(self, other):
        return self.f < other.f


class Graph:

    graph_dict = {}
    weights = {}

    def addEdge(self, node, neighbour, dist):
        if node not in self.graph_dict:
            self.graph_dict[node] = [neighbour]
            self.weights[node] = [dist]

        else:
            self.graph_dict[node].append(neighbour)
            self.weights[node].append(dist)


global g
g = Graph()

global nodes
nodes = []


class PRM:
    global g
    global nodes
    global path

    def __init__(self):
        self.sample_area = (0, 6)
        self.r = 4
        self.n = 150

    def construct(self):
        i = 0
        while i < self.n:
            p = sampler(self.sample_area)
            if not chk_in_obs(p):
                i += 1
                nodes.append(p)
        for i in range(len(nodes)):
            for j in range(len(nodes)):
                if i != j:
                    dist = math.sqrt(
                        (nodes[i][0] - nodes[j][0]) ** 2
                        + (nodes[i][1] - nodes[j][1]) ** 2
                    )
                    if dist < self.r:
                        if not chk_obs(i, j):
                            if (i not in g.graph_dict) or (j not in g.graph_dict[i]):
                                g.addEdge(i, j, dist)
                            if (j not in g.graph_dict) or (i not in g.graph_dict[j]):
                                g.addEdge(j, i, dist)

    @staticmethod
    def visualize_tree(graph, obstacle_list):
        global nodes
        plt.clf()
        for node in graph.graph_dict:
            for neighbour in graph.graph_dict[node]:
                plt.plot(
                    [nodes[int(node)][0], nodes[neighbour][0]],
                    [nodes[int(node)][1], nodes[neighbour][1]],
                )

        for obstacle in obstacle_list:
            circle1 = plt.Circle((obstacle[0], obstacle[1]), 0.25)
            plt.gcf().gca().add_artist(circle1)

        plt.show()

    def plan(self, beg, end):
        global g
        global nodes
        open_ = []
        closed = []

        start_node = Node("start", None)

        closed.append(start_node)
        min_dist = float("inf")
        for node in nodes:
            dist = math.sqrt((node[0] - beg[0]) ** 2 + (node[1] - beg[1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_index = nodes.index(node)
        nxt = Node(min_index, start_node)
        open_.append(nxt)

        min_dist = float("inf")
        for node in nodes:
            dist = math.sqrt((node[0] - end[0]) ** 2 + (node[1] - end[1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_index = nodes.index(node)

        goal = min_index
        goal_node = Node(goal, None)

        while len(open_) > 0:
            open_.sort()
            current_node = open_.pop(0)
            closed.append(current_node)
            if current_node == goal_node:
                path.header.frame_id = "path"
                pose = PoseStamped()
                pose.pose.position.x = end[0]
                pose.pose.position.y = end[1]
                pose.pose.position.z = 0
                path.poses.append(pose)
                while current_node != start_node:
                    pose = PoseStamped()
                    pose.pose.position.x = nodes[int(current_node.name)][0]
                    pose.pose.position.y = nodes[int(current_node.name)][1]
                    pose.pose.position.z = 0
                    path.poses.append(pose)
                    current_node = current_node.parent
                return True
            neighbors = g.graph_dict[int(current_node.name)]
            for key in neighbors:
                neighbor = Node(key, current_node)

                if neighbor in closed:
                    continue

                a = g.graph_dict[int(current_node.name)].index(key)
                neighbor.g = current_node.g + g.weights[int(current_node.name)][a]
                neighbor.h = math.sqrt(
                    (nodes[int(goal)][0] - nodes[int(neighbor.name)][0]) ** 2
                    + (nodes[int(goal)][1] - nodes[int(neighbor.name)][1]) ** 2
                )
                neighbor.f = neighbor.g + neighbor.h

                if add_to_open(open_, neighbor) == True:
                    open_.append(neighbor)

        return False


rospy.init_node("path_planner")
sub = rospy.Subscriber("obstacle", Path, callback)
pub = rospy.Publisher("path", Path, queue_size=5)
rospy.spin()
