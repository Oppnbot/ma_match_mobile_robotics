#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# Bugfixes:
# todo: when waiting in a cell, check for collision with later robots
# todo: start/stop positioning are not scaling
# todo: don't communicate via magic numbers, use publisher and subscribers

# Code Quality:
# todo: replace waypoints by ros messages
# todo: distribute code to multiple nodes

# Additional Features:
# todo: option for wider paths to fit robot width (+map scaling)
# todo: improve occupation time calculations with safety margin etc
# todo: assign new priorities when there is no viable path
# todo: assign new priorities when a path took too long (?)
# todo: implement path following
# todo: distribute calculation to multiple robots for O{(n-1)!} instead of O{n!}

# Other:
# todo: skip visited nodes ?
# todo: test in different scenarios
# todo: adjust to the robots real dynamics (?)
# todo: improve overall computing performance


from __future__ import annotations
from operator import index
from pty import spawn

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import heapq
from commons import TrajectoryData, Waypoint
from visualization import fb_visualizer
#from formation_builder.srv import transformation
from formation_builder.srv import TransformPixelToWorld, TransformPixelToWorldRequest, TransformPixelToWorldResponse


class Spawner:
    def __init__(self)->None:
        # -------- CONFIG START --------
        self.path_finder_count : int = 4
        #self.starting_positions : list[tuple[int, int]] =  [(1, 1), (3, 1), (5, 1), (7, 1)]
        #self.goal_positions: list[tuple[int, int]] =       [(70, 38), (62, 17), (60, 39), (52, 12)]
        
        self.starting_positions : list[tuple[int, int]] =  [(20, 20), (35, 48), (25, 19), (25, 50)]
        self.goal_positions: list[tuple[int, int]] =       [(70, 38), (53, 11), (60, 39), (52, 12)]

        # --------- CONFIG END ---------
        rospy.init_node('mapf')
        

        self.path_finders : list[WavefrontExpansionNode] = []
        self.cv_bridge : CvBridge = CvBridge()
        self.occupation: dict[tuple[int, int], list[tuple[float, float]]] = {} #key: x- and y-val of the gridcell. value: list of timings with "occupied from" and "occupied until"
        for i in range(self.path_finder_count):
            self.path_finders.append(WavefrontExpansionNode(i))
        

        rospy.loginfo("waiting for transformation service...")
        rospy.wait_for_service('pixel_to_world')
        rospy.loginfo("transformation service is running!")
        
        rospy.Subscriber('/formation_builder/map', Image, self.map_callback)

        return None
    

    def init_occupation_dict(self, width:int, height : int) -> None:
        for w in range(width):
            for h in range(height):
                self.occupation[(w, h)] = []
        return None
    

    def update_occupation_dict(self, trajectory: TrajectoryData) -> None:
        for waypoint in trajectory.waypoints:
            self.occupation[waypoint.pixel_pos].append((waypoint.occupied_from, waypoint.occuped_until))
        return None


    def map_callback(self, img_msg : Image) -> None:
        grid : np.ndarray = self.map_to_grid(img_msg)
        paths : list[TrajectoryData] = []

        fb_visualizer.clear_image(grid.shape[0], grid.shape[1])
        fb_visualizer.draw_obstacles(grid)

        self.init_occupation_dict(grid.shape[0], grid.shape[1])
        start_time = start_time = time.time()

        for index, path_finder in enumerate(self.path_finders):
            if index >= len(self.starting_positions) or index >= len(self.goal_positions):
                rospy.logwarn("there are not enough starting/goal positions for the chosen ammount of planners.")
                break
            trajectory : TrajectoryData = path_finder.path_finder(grid, self.starting_positions[index], self.goal_positions[index], self.occupation)
            paths.append(trajectory)

            self.update_occupation_dict(trajectory)

            fb_visualizer.draw_path(trajectory, self.path_finder_count)
            fb_visualizer.draw_start_and_goal(trajectory, self.path_finder_count)
            fb_visualizer.publish_image()
        end_time = time.time()

        rospy.loginfo(f"------------------ Done! ({end_time-start_time:.3f}s) ------------------ ")
        fb_visualizer.show_live_path(paths)
        return None



    def map_to_grid(self, map : Image) -> np.ndarray:
        img = self.cv_bridge.imgmsg_to_cv2(map, desired_encoding="mono8")
        grid : np.ndarray = np.array(img) // 255
        return grid
    


class WavefrontExpansionNode:
    def __init__(self, planner_id : int = 0):
        # -------- CONFIG START --------
        self.allow_diagonals : bool = False
        self.check_dynamic_obstacles : bool = True
        self.dynamic_visualization : bool = False # publishes timing map after every step, very expensive
        # -------- CONFIG END --------
        
        self.id: int = planner_id
        self.grid_resolution = 2.0
        self.cv_bridge = CvBridge()
        self.point_size : int = 2

        

        return None


    def process_image(self, img):
        grid : np.ndarray = np.array(img) // 255
        return grid.tolist()    

    def path_finder(self, static_obstacles: np.ndarray, start_pos: tuple[int, int], goal_pos: tuple[int, int], occupation: dict[tuple[int, int], list[tuple[float, float]]]) -> TrajectoryData:
        rospy.loginfo(f"Planner {self.id} Starting wavefront expansion")
        start_time = time.time()

        heap: list[tuple[float, tuple[int, int]]] = [(0, start_pos)]
        rows: int = static_obstacles.shape[0]
        cols: int = static_obstacles.shape[1]

        direct_neighbors: list[tuple[int, int]] = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        diagonal_neighbors: list[tuple[int, int]] = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        neighbors: list[tuple[int, int]] = direct_neighbors
        if self.allow_diagonals:
            neighbors += diagonal_neighbors

        timings: np.ndarray = np.zeros((rows, cols)) - 1
        predecessors: dict[tuple[int, int], tuple[int, int]] = {}

        iterations: int = 0

        timings[start_pos[0], start_pos[1]] = 0.0

        
        #* --- Generate Timings ---
        while heap:
            iterations += 1
            current_cost, current_element = heapq.heappop(heap)

            if self.dynamic_visualization:
                fb_visualizer.draw_timings(timings, static_obstacles, start_pos, goal_pos)

            if iterations % 1000 == 0:
                rospy.loginfo(f"planner {self.id}: {iterations} iterations done!")

            if iterations > 500_000:
                rospy.logwarn(f"planner {self.id}: breaking because algorithm reached max iterations")
                break

            if current_element == goal_pos:
                rospy.loginfo(f"planner {self.id}: Reached the goal after {iterations} iterations")
                break

            #current_cost = timings[current_element[0], current_element[1]]

            for x_neighbor, y_neighbor in neighbors:
                x, y = current_element[0] + x_neighbor, current_element[1] + y_neighbor
                if 0 <= x < rows and 0 <= y < cols and static_obstacles[x, y] != 0: # check for static obstacles / out of bounds
                    driving_cost = current_cost + (1 if abs(x_neighbor + y_neighbor) == 1 else 1.41422)

                    if self.check_dynamic_obstacles and occupation[(x,y)]: # check for dynamic obstacles
                        is_occupied : bool = False
                        occupation_list : list[tuple[float, float]] = occupation[(x,y)]
                        for occ_from, occ_until in occupation_list:
                            if occ_from <= driving_cost <= occ_until:
                                #rospy.logwarn(f"robots {self.id} path crosses at ({x, y}) from after {driving_cost}. it is occupied between {occ_from} to {occ_until} ")
                                heapq.heappush(heap, (occ_until, (x, y)))
                                is_occupied = True
                                break
                        if is_occupied:
                            continue
                    # cell isnt occupied -> add it to heap
                    if driving_cost < timings[x, y] or timings[x, y] < 0:
                        timings[x, y] = driving_cost
                        predecessors[(x, y)] = current_element
                        heapq.heappush(heap, (driving_cost, (x, y)))

        rospy.loginfo(f"planner {self.id}: stopped after a total of {iterations} iterations")
        end_time = time.time()
        elapsed_time = end_time - start_time
        rospy.loginfo(f"planner {self.id}: planned path in {elapsed_time:.3f}s.")
        

        #* --- Reconstruct Path ---
        waypoints : list[Waypoint] = []

        # Reconstruct path from goal to start using the predecessor of each node
        current_node: tuple[int, int] = goal_pos
        previous_node: tuple[int, int] | None = None
        while current_node != start_pos:
            
            if current_node in predecessors.keys():
                current_node = predecessors[current_node]
                waypoint : Waypoint = Waypoint()
                waypoint.pixel_pos = current_node
                waypoint.occupied_from = timings[current_node[0], current_node[1]]
                if previous_node is None:
                    waypoint.occuped_until = float('inf')
                else:
                    #todo: add some uncertainty compensation here
                    #waypoint.occuped_until = timings[previous_node[0], previous_node[1]] # unmodified
                    waypoint.occuped_until = timings[previous_node[0], previous_node[1]] + 2            # defined snake length
                    #waypoint.occuped_until = (timings[previous_node[0], previous_node[1]] + 1) * 1.1   # snakes get longer over time -> uncertainty grows
                waypoints.append(waypoint)
                previous_node = current_node

                if self.dynamic_visualization:
                    fb_visualizer.draw_timings(timings, static_obstacles, start_pos, goal_pos, waypoints)
                
            else:
                rospy.logerr(f"Cant reconstruct path since {current_node} is not in keys; Path might be unvalid.")
                break
        start_point : Waypoint = Waypoint()
        start_point.occupied_from = 0
        start_point.occuped_until = waypoints[-1].occupied_from # todo: add uncertainty here
        start_point.pixel_pos = start_pos
        waypoints.append(start_point)
        waypoints.reverse()

        trajectory_data : TrajectoryData = TrajectoryData(self.id, waypoints)

        rospy.loginfo(f"planner {self.id}: shortest path consists of {len(waypoints)} nodes with a cost of {timings[goal_pos[0], goal_pos[1]]}")

        # Transform Path from Pixel-Space to World-Space for visualization and path following
        transform_pixel_to_world = rospy.ServiceProxy('pixel_to_world', TransformPixelToWorld)
        for waypoint in trajectory_data.waypoints:
            response : TransformPixelToWorldResponse = transform_pixel_to_world(waypoint.pixel_pos[0], waypoint.pixel_pos[1])
            waypoint.world_pos = (response.x_world, response.y_world)
        
        # Visualization
        fb_visualizer.draw_timings(timings, static_obstacles, start_pos, goal_pos, trajectory_data.waypoints)

        return trajectory_data
    
    

if __name__ == '__main__':
    spawner : Spawner = Spawner()
    rospy.spin()

