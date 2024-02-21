#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
from operator import index

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import heapq
import colorsys

class PathData():
    def __init__(self, planner_id:int, timings:np.ndarray, path:list[tuple[int, int]] = []):
        self.planner_id : int = planner_id
        self.path : list[tuple[int, int]] = path
        self.timings : np.ndarray = timings
        self.start: tuple[int, int]
        self.goal: tuple[int, int]
        if len(path) > 0:
            self.start = path[-1]
            self.goal = path[0]

class Spawner:
    def __init__(self)->None:
        self.path_finder_count : int = 4

        rospy.init_node('mapf')
        self.cv_bridge = CvBridge()

        self.path_finders : list[WavefrontExpansionNode] = []


        self.starting_positions : list[tuple[int, int]] =   [(25, 12), (25, 16), (20, 20), (7, 30)]
        self.goal_positions: list[tuple[int, int]] =       [(48, 37), (48, 39), (48, 50), (55, 20)]

        self.occupation: dict[tuple[int, int], list[tuple[float, float]]] = {} #key: x- and y-val of the gridcell. value: list of timings with "occupied from" and "occupied until"

        for i in range(self.path_finder_count):
            self.path_finders.append(WavefrontExpansionNode(i))

        rospy.Subscriber('/formation_builder/map', Image, self.map_callback)
        self.image_pub = rospy.Publisher(f"/path_image", Image, queue_size=1, latch=True)

        return None
    

    def init_occupation_dict(self, width:int, height : int) -> None:
        for w in range(width):
            for h in range(height):
                self.occupation[(w, h)] = []
        return None
    

    def update_occupation_dict(self, path_data: PathData) -> None:
        #! this function is somewhat buggy since it blocks the starting pos instead of the goal pos. may want to reorder that list
        for index, point in enumerate(path_data.path):
            occ_from : float = path_data.timings[point[0], point[1]]
            occ_until : float = float('inf')
            if index < len(path_data.path)-1:
                occ_until : float = path_data.timings[path_data.path[index+1][0], path_data.path[index+1][1]] + 2 #! might remove this +1
            self.occupation[point].append((occ_from, occ_until))
        return None
    

    def map_callback(self, img_msg : Image) -> None:
        grid : np.ndarray = self.map_to_grid(img_msg)
        paths : list[PathData] = []

        result_image : np.ndarray = self.clear_image(grid.shape[0], grid.shape[1])
        result_image = self.draw_obstacles(result_image, grid)

        self.init_occupation_dict(grid.shape[0], grid.shape[1])
        start_time = start_time = time.time()

        for index, path_finder in enumerate(self.path_finders):
            if index >= len(self.starting_positions) or index >= len(self.goal_positions):
                rospy.logwarn("there are not enough starting/goal positions for the chosen ammount of planners.")
                break
            path : PathData = path_finder.wavefront_expansion(grid, self.starting_positions[index], self.goal_positions[index], self.occupation)
            paths.append(path)

            self.update_occupation_dict(path)

            result_image = self.draw_path(result_image, path)
            result_image = self.draw_start_and_goal(result_image, path)
            self.publish_image_matrix(result_image)

        end_time = time.time()

        rospy.loginfo(f"------------------ Done! ({end_time-start_time:.3f}s) ------------------ ")
        return None



    def map_to_grid(self, map : Image) -> np.ndarray:
        img = self.cv_bridge.imgmsg_to_cv2(map, desired_encoding="mono8")
        grid : np.ndarray = np.array(img) // 255
        return grid
    

    def clear_image(self, width: int, height: int) -> np.ndarray:
        image_matrix = np.ones((width, height, 3), dtype=np.uint8)*255 # white image as default
        return image_matrix
    

    def draw_obstacles(self, image_matrix:np.ndarray, obstacles: np.ndarray) -> np.ndarray:
        for i in range(image_matrix.shape[0]):
            for j in range(image_matrix.shape[1]):
                if obstacles[i, j] == 0:
                    image_matrix[i, j] = (0, 0, 0)
        return image_matrix


    def draw_path(self, image_matrix:np.ndarray, path_data : PathData) -> np.ndarray:
        color = self.generate_distinct_colors(self.path_finder_count, path_data.planner_id, value=0.5)
        
        for point in path_data.path:
            current_val = image_matrix[point[0], point[1]]
            if all(x == 0 or x == 255 for x in current_val):
                image_matrix[point[0], point[1]] = color
            else:
                r,g,b = color
                new_color = (current_val[0] + r//2, current_val[1] + g//2, current_val[2] + b//2)
                image_matrix[point[0], point[1]] = new_color
        return image_matrix
    

    def draw_start_and_goal(self, image_matrix:np.ndarray, path_data : PathData) -> np.ndarray:
        color = self.generate_distinct_colors(self.path_finder_count, path_data.planner_id)
        image_matrix[path_data.goal[0], path_data.goal[1]] = color
        image_matrix[path_data.start[0], path_data.start[1]] = color
        return image_matrix
    

    def publish_image_matrix(self, image_matrix:np.ndarray) -> None:
        rospy.loginfo("publishing a new image")
        image_cv = np.uint8(image_matrix)
        image_msg : Image = self.cv_bridge.cv2_to_imgmsg(image_cv, encoding="rgb8")
        #cv2.imshow("path", image_cv)
        #cv2.waitKey(0)
        self.image_pub.publish(image_msg)
        return None
    

    def generate_distinct_colors(self, num_colors:int, index:int, saturation : float = 1.0, value: float = 1.0) -> tuple[np.uint8, np.uint8 ,np.uint8]:
        # num colors: total number of colors
        # index = id of the path or whatever
        # saturation and value should be in a range from 0 to 1
        hue_delta : float = 360.0 / num_colors
        hue = (index * hue_delta) % 360
        r, g, b = colorsys.hsv_to_rgb(hue / 360.0, saturation, value)
        r, g, b = np.uint8(r * 255), np.uint8(g * 255), np.uint8(b * 255)
        return (r, g, b)

    


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
                self.draw_timings(timings, static_obstacles, start_pos, goal_pos)

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
        self.draw_timings(timings, static_obstacles, start_pos, goal_pos, path)
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
    

    def draw_timings(self, grid: np.ndarray, obstacles: np.ndarray, start: tuple[int, int], goal: tuple[int, int], path:list[tuple[int, int]]=[]) -> None:
        #rospy.logerr("draw timings is a deprecated function. please refactor your code")
        min_val = np.min(grid)
        max_val  = np.max(grid)
        image_matrix = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                val = grid[i, j]
                if obstacles[i][j] == 0:
                    image_matrix[i, j] = (0, 0, 0)  # black for obstacles
                elif val == -1:
                    image_matrix[i, j] = (255, 255, 255)  # white for non-visited spaces
                else:
                    blue_value = int(200 * (val - min_val) / (max_val - min_val)) + 55
                    image_matrix[i, j] = (255 - blue_value, 0, blue_value)  # red/blue depending on timing
        # path:
        for point in path:
            image_matrix[point[0], point[1]] = (0, 125 , 0)
        # start and goal:
        image_matrix[goal[0], goal[1]] = (255, 0, 0)
        image_matrix[start[0], start[1]] = (0, 255, 0)

        image_msg : Image = self.cv_bridge.cv2_to_imgmsg(image_matrix, encoding="rgb8")
        #rospy.loginfo("published a new image")
        timing_pub: rospy.Publisher = rospy.Publisher("/timing_image", Image, queue_size=1, latch=True)
        timing_pub.publish(image_msg)
        return None





if __name__ == '__main__':
    spawner : Spawner = Spawner()
    rospy.spin()

