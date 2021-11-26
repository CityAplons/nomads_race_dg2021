import open3d as o3d
import numpy as np
import rospy
from utils.helpers import Node

class AStar:
    
    def __init__(self, map: o3d.geometry.VoxelGrid) -> None:
        self.map = map

    def return_path(self, current: Node):
        path = []
        node = current
        while node is not None:
            path.append(node.position)
            node = node.parent
        return path[::-1]
    
    def search(self, cost, start, end):
        start_node = Node(None, start)
        end_node = Node(None, end)

        to_visit = []
        visited = []
        to_visit.append(start_node)

        outer_iterations = 0
        max_iterations = 5000

        movements = np.array([
            [0, 0, -1],
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, 1],
            [0, 1, 0],
            [1, 0, 0],
        ])

        nz = 19
        nx_max = start_node.position[0] + 30 
        nx_min = start_node.position[0] - 10 
        ny_max = start_node.position[1] + 18 
        ny_min = start_node.position[1] - 18 

        while len(to_visit):
            outer_iterations += 1
            current_node = to_visit[0]
            current_index = 0
            for index, item in enumerate(to_visit):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
            if outer_iterations > max_iterations:
                rospy.logwarn("Path not found")
                return self.return_path(current_node)
            to_visit.pop(current_index)
            visited.append(current_node)

            if current_node == end_node:
                rospy.logwarn("Path found")
                return self.return_path(current_node)
            
            children = []
            for step in movements:
                node_position = current_node.position + step

                if (
                    node_position[2] < -0.1 or node_position[2] > nz or
                    node_position[0] < nx_min or node_position[0] > nx_max or
                    node_position[1] < ny_min or node_position[1] > ny_max
                    
                ):
                    continue
                
                collision = self.map.check_if_included(o3d.utility.Vector3dVector(np.array([node_position])))[0]
                if collision:
                    continue

                new_node = self.Node(current_node, node_position)
                children.append(new_node)
            
            for child in children:
                if len([visited_child for visited_child in visited if visited_child == child]):
                    continue
                    
                child.g = current_node.g + cost
                p1 = child.position
                p2 = end_node.position
                child.h = np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 +(p1[2]-p2[2])**2)
                child.f = child.g + child.h

                if len([i for i in to_visit if child == i and child.g > i.g]):
                    continue

                to_visit.append(child)

