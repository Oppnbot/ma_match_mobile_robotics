#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# todo: skip visited nodes ?
# todo: pfadsuche austauschen, stattdessen last visited tracken
# todo: fix those start-/stop occupation times
# todo: when waiting in a cell, check for collision with later robots
# todo: option for wider paths to fit robot width (+map scaling)
# todo: weird bug when not allowing for 8neighborhood in pf
# todo: improve occupation time calculations with safety margin etc
# todo: test in different scenarios
# todo: assign new priorities when there is no viable path
# todo: assign new priorities when a path took too long (?)
# todo: implement path following
# todo: adjust to the robots real dynamics (?)
# todo: improve and test performance, maybe distribute the system to multiple pcs


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
from commons import PathData
from visualization import fb_visualizer
#from formation_builder.srv import transformation
from formation_builder.srv import TransformPixelToWorld, TransformPixelToWorldRequest, TransformPixelToWorldResponse


class Spawner:
    def __init__(self)->None:
        # -------- CONFIG START --------
        self.path_finder_count : int = 4
        #self.starting_positions : list[tuple[int, int]] =  [(1, 1), (3, 1), (5, 1), (7, 1)]
        #self.goal_positions: list[tuple[int, int]] =       [(70, 38), (62, 17), (60, 39), (52, 12)]
        
        self.starting_positions : list[tuple[int, int]] =  [(20, 20), (34, 25), (25, 20), (30, 45)]
        self.goal_positions: list[tuple[int, int]] =       [(70, 38), (62, 17), (60, 39), (52, 12)]

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
    

    def update_occupation_dict(self, path_data: PathData) -> None:
        #! this function is somewhat buggy since it blocks the starting pos instead of the goal pos. may want to reorder that list
        for index, point in enumerate(path_data.path_pixel):
            occ_from : float = path_data.timings[point[0], point[1]]
            occ_until : float = float('inf')
            if index < len(path_data.path_pixel)-1:
                occ_until : float = path_data.timings[path_data.path_pixel[index+1][0], path_data.path_pixel[index+1][1]] + 1 #! might remove this +1
            self.occupation[point].append((occ_from, occ_until))
        return None
    

    def map_callback(self, img_msg : Image) -> None:
        grid : np.ndarray = self.map_to_grid(img_msg)
        paths : list[PathData] = []

        fb_visualizer.clear_image(grid.shape[0], grid.shape[1])
        fb_visualizer.draw_obstacles(grid)

        self.init_occupation_dict(grid.shape[0], grid.shape[1])
        start_time = start_time = time.time()

        for index, path_finder in enumerate(self.path_finders):
            if index >= len(self.starting_positions) or index >= len(self.goal_positions):
                rospy.logwarn("there are not enough starting/goal positions for the chosen ammount of planners.")
                break
            path : PathData = path_finder.wavefront_expansion(grid, self.starting_positions[index], self.goal_positions[index], self.occupation)
            paths.append(path)

            self.update_occupation_dict(path)

            fb_visualizer.draw_path(path, self.path_finder_count)
            fb_visualizer.draw_start_and_goal(path, self.path_finder_count)
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
    def __init__(self, planner_id:int=0):
        self.id: int = planner_id
        self.grid_resolution = 2.0
        self.cv_bridge = CvBridge()
        self.point_size : int = 2

        self.allow_diagonals : bool = False
        self.check_dynamic_obstacles : bool = True
        self.dynamic_visualization : bool = False # publishes timing map after every step, very expensive

        return None


    def process_image(self, img):
        grid : np.ndarray = np.array(img) // 255
        return grid.tolist()    

    def wavefront_expansion(self, static_obstacles: np.ndarray, start_pos: tuple[int, int], goal_pos: tuple[int, int], occupation: dict[tuple[int, int], list[tuple[float, float]]]) -> PathData:
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
        iterations: int = 0

        timings[start_pos[0], start_pos[1]] = 0.0

        while heap:
            iterations += 1
            current_cost, current_element = heapq.heappop(heap)

            if self.dynamic_visualization:
                fb_visualizer.draw_timings(timings, static_obstacles, start_pos, goal_pos)

            if iterations % 1000 == 0:
                rospy.loginfo(f"planner {self.id}: {iterations} iterations done!")

            if iterations > 500000:
                rospy.logwarn(f"planner {self.id}: breaking because algorithm reached max iterations")
                break

            if current_element == goal_pos:
                rospy.loginfo(f"planner {self.id}: Reached the goal after {iterations} iterations")
                break

            #current_cost = timings[current_element[0], current_element[1]]

            for x_neighbor, y_neighbor in neighbors:
                x, y = current_element[0] + x_neighbor, current_element[1] + y_neighbor
                if 0 <= x < rows and 0 <= y < cols and static_obstacles[x, y] != 0: # check for static obstacles / out of bounds
                    driving_cost = current_cost + (1 if abs(x_neighbor + y_neighbor) == 1 else 1.41421366)

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
                        heapq.heappush(heap, (driving_cost, (x, y)))

        rospy.loginfo(f"planner {self.id}: stopped after a total of {iterations} iterations")
        end_time = time.time()
        elapsed_time = end_time - start_time
        rospy.loginfo(f"planner {self.id}: planned path in {elapsed_time:.3f}s.")

        path: list[tuple[int, int]] = self.find_path(timings, start_pos, goal_pos)
        rospy.loginfo(f"planner {self.id}: shortest path consists of {len(path)} nodes with a cost of {timings[goal_pos[0], goal_pos[1]]}")
        path_data: PathData = PathData(self.id, timings, path)
        path_data.start = start_pos
        path_data.goal = goal_pos
        path_data.timings = timings

        
        transform_pixel_to_world = rospy.ServiceProxy('pixel_to_world', TransformPixelToWorld)
        for point in path:
            response : TransformPixelToWorldResponse = transform_pixel_to_world(point[0], point[1])
            path_data.path_world.append((response.x_world, response.y_world))   
        fb_visualizer.draw_timings(timings, static_obstacles, start_pos, goal_pos, path)
        return path_data


    def find_path(self, timings: np.ndarray, start_pos:tuple[int, int], goal_pos:tuple[int, int]) -> list[tuple[int, int]]:
        rospy.loginfo(f"planner {self.id} searching for the shortest path...")
        path : list[tuple[int, int]] = []
        next_pos : tuple[int, int] = goal_pos

        direct_neighbors : list[tuple[int, int]] = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        diagonal_neighbors: list[tuple[int, int]] = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        neighbors: list[tuple[int, int]] = direct_neighbors
        if self.allow_diagonals or True:
            neighbors += diagonal_neighbors
        rows = len(timings)
        cols = len(timings[0])
        while next_pos != start_pos:
            lowest_timing : float = float('inf')
            lowest_neigbor: tuple[int, int]

            for x_neighbor, y_neighbor in neighbors:
                x, y = next_pos[0] + x_neighbor, next_pos[1] + y_neighbor
                if 0 <= x < rows and 0 <= y < cols and -1 < timings[x, y] < lowest_timing:
                    lowest_timing = timings[x, y]
                    lowest_neigbor = (x, y)
            if lowest_timing == float('inf'):
                # This error means that goal may be unreachable. should not occur if wavefront generation succeeded
                rospy.logerr(f"planner {self.id} pathfinding failed! there is no valid neighbor")
                break
            next_pos = lowest_neigbor
            if lowest_neigbor in path:
                # This error may occur when using different neighborhood metrics in path finding and wavefront generation
                rospy.logerr(f"planner {self.id} pathfinding failed! Stuck in a dead end!")
                break
            path.append(lowest_neigbor)
        return path
    

if __name__ == '__main__':
    spawner : Spawner = Spawner()
    rospy.spin()

