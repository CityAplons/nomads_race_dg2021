import copy
import math
import bezier
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np

from utils.helpers import *

# # RRT* Pseudo Code

# # Rad = r
# # G(V,E) //Graph containing edges and vertices
# # For itr in range(0…n)
# #     Xnew = RandomPosition()
# #     If Obstacle(Xnew) == True, try again
# #     Xnearest = Nearest(G(V,E),Xnew)
# #     Cost(Xnew) = Distance(Xnew,Xnearest)
# #     Xbest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
# #     Link = Chain(Xnew,Xbest)
# #     For x’ in Xneighbors
# #         If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
# #             Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
# #             Parent(x’) = Xnew
# #             G += {Xnew,x’}
# #     G += Link
# # Return G


class RRTStar:

    def __init__(
            self,
            start,
            obstacles: o3d.geometry.VoxelGrid,
            radius: float = 1.0,
            area: Area = None,
            resolution: int = 50,
            it_limit: int = 500) -> None:
        if area is None:
            self.area = Area()
        else:
            self.area = area
        self.rad = radius
        self.counter = 0
        self.limit = it_limit
        self.start = start
        self.obstacles = obstacles
        self.G = [Node(None, self.start)]
        self.goal = None
        self.res = resolution

    def form_path(self) -> list:
        goal_ind = self.search_best_goal_node()
        if goal_ind is not None:
            path = [self.goal]
            node = self.G[goal_ind]
            while node.parent is not None:
                path.append(node.position)
                node = self.G[node.parent]
            path.append(node.position)
            return path[::-1]
        else:
            return None

    def steer(self, start: Node, end: Node):
        nodes = np.array([
            start.position,
            end.position
        ]).T
        curve = bezier.Curve.from_nodes(nodes)
        t = np.linspace(0.0, 1.0, self.res)
        return curve.evaluate_multi(t).T, curve.length

    def choose(self, neighbors_id, nearest, new, steering, steering_cost):
        res = nearest
        node_min = self.G[nearest]
        cost_min = node_min.h + steering_cost

        for id in neighbors_id:
            path, cost = self.steer(self.G[id], new)
            if self.is_obstacle(path):
                continue
            else:
                updated_cost = self.G[id].h + cost
                if updated_cost < new.h and updated_cost < cost_min:
                    res = id
                    node_min = self.G[id]
                    cost_min = updated_cost
        return res

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.G:
            if node.parent is not None and self.G[node.parent] == parent_node:
                node.h = parent_node.h + \
                    euclidean_distance(parent_node.position, node.position)
                self.propagate_cost_to_leaves(node)

    def rewire(self, neighbour_ids, node_min_id, new: Node):
        for id in neighbour_ids:
            path, cost = self.steer(new, self.G[id])
            if self.is_obstacle(path) and (new.h + cost) >= self.G[id].h:
                continue
            self.G[id] = new
            self.propagate_cost_to_leaves(new)

    def find_nearest(self, node: Node):
        dList = [euclidean_distance(node_it.position, node.position)
                 for node_it in self.G]
        minIndex = dList.index(min(dList))
        return minIndex

    def find_neighbours(self, node: Node):
        r = self.rad
        dList = [euclidean_distance(node_it.position, node.position)
                 for node_it in self.G]
        near_inds = [dList.index(d) for d in dList if d <= self.rad]
        return near_inds

    def is_obstacle(self, point_vector):
        collisions = self.obstacles.check_if_included(
            o3d.utility.Vector3dVector(point_vector))
        return np.any(collisions)

    def search(self, goal):
        min_v, max_v = self.area.get_vectors()
        self.goal = goal
        while self.counter < self.limit:
            self.counter += 1
            pos_new = np.array([
                np.random.uniform(min_v[0], max_v[0]),
                np.random.uniform(min_v[1], max_v[1]),
                np.random.uniform(min_v[2], max_v[2])
            ])
            if self.is_obstacle([pos_new]):
                continue
            else:
                node_new = Node(None, pos_new)
                nearest_node = self.find_nearest(node_new)
                temp_path, temp_cost = self.steer(
                    self.G[nearest_node], node_new)
                if self.is_obstacle(temp_path):
                    continue
                else:
                    neighbour_ids = self.find_neighbours(node_new)
                    node_min_id = self.choose(
                        neighbour_ids, nearest_node, node_new, temp_path, temp_cost)
                    if node_min_id != nearest_node:
                        self.rewire(neighbour_ids, node_min_id, node_new)
                    node_new.parent = node_min_id
                    node_new.h = self.G[node_new.parent].h + temp_cost

                    self.G.append(node_new)

                    if np.allclose(pos_new, goal, atol=0.2):
                        return self.form_path()
                        
            if self.counter%100 == 0:
                print("[Planner] Cycles %d, length: %d" %
                    (self.counter, len(self.G)))
        return self.form_path()

    def search_best_goal_node(self):
        nodes = copy.deepcopy(self.G)
        dist_to_goal_list = [
            euclidean_distance(n.position, self.goal) for n in nodes
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.rad
        ]

        safe_goal_inds = []
        temp = Node(None, self.goal)
        for goal_ind in goal_inds:
            temp_path,_ = self.steer(nodes[goal_ind], temp)
            if not self.is_obstacle(temp_path):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([nodes[i].h for i in safe_goal_inds])
        for i in safe_goal_inds:
            if nodes[i].h == min_cost:
                return i
        return None

    def smooth_path(self, path):
        new_path = []
        for i in range(1, len(path)):
            nodes = np.array([
                path[i-1],
                path[i]
            ]).T
            curve = bezier.Curve.from_nodes(nodes)
            t = np.linspace(0.0, 1.0, self.res)
            new_path.extend(list(curve.evaluate_multi(t).T))
        return new_path
