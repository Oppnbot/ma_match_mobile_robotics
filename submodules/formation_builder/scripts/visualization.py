#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
from operator import index

from yaml import Mark

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import colorsys
from commons import PathData
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time

class Visualization():
    def __init__(self) -> None:
        self.cv_bridge : CvBridge = CvBridge()
        self.debug_image_pub = rospy.Publisher(f"/formation_builder/debug_image", Image, queue_size=1, latch=True)
        self.current_image_data : np.ndarray = np.zeros((100, 100))
        return None
    

    def generate_distinct_colors(self, num_colors:int, index:int, saturation : float = 1.0, value: float = 1.0) -> tuple[np.uint8, np.uint8 ,np.uint8]:
        """
        This function generates a specified number of colors with different hue values. The hue values are chosen in a way that maximizes the contrast between the different colors.

        :param num_colors: the total ammount of possible colors. Should be equal to the total number of agents you want to visualize.
        :param index: use a number between [0...num_colors[. usually this is equal to the agent id.
        :param saturation: Provide the saturation of the generated color in the range from 0.0 -> 1.0 (optional)
        :param value: Provide the value of the generated color in the range from 0.0 -> 1.0 (optional)
        :return: returns a color tuple with the format (red, green, blue) with a range of 0 -> 255 for each value
        """
        hue_delta : float = 360.0 / num_colors
        hue = (index * hue_delta) % 360
        r, g, b = colorsys.hsv_to_rgb(hue / 360.0, saturation, value)
        r, g, b = np.uint8(r * 255), np.uint8(g * 255), np.uint8(b * 255)
        return (r, g, b)


    def clear_image(self, width: int, height: int) -> np.ndarray:
        image_matrix = np.ones((width, height, 3), dtype=np.uint8)*255 # white image as default
        self.current_image_data = image_matrix
        return image_matrix
    

    def draw_obstacles(self, obstacles: np.ndarray, image_matrix:np.ndarray|None = None) -> np.ndarray:
        if image_matrix is None:
            image_matrix = self.current_image_data
        for i in range(image_matrix.shape[0]):
            for j in range(image_matrix.shape[1]):
                if obstacles[i, j] == 0:
                    image_matrix[i, j] = (0, 0, 0)
        self.current_image_data = image_matrix
        return image_matrix


    def draw_path(self, path_data : PathData, number_of_agents : int, image_matrix:np.ndarray|None = None) -> np.ndarray:
        if image_matrix is None:
            image_matrix = self.current_image_data
        color = self.generate_distinct_colors(number_of_agents, path_data.planner_id, value=0.7)
        for point in path_data.path_pixel:
            current_val = image_matrix[point[0], point[1]]
            if all(x == 0 or x == 255 for x in current_val):
                image_matrix[point[0], point[1]] = color
            else:
                r,g,b = color
                new_color = (current_val[0] + r//2, current_val[1] + g//2, current_val[2] + b//2)
                image_matrix[point[0], point[1]] = new_color
        self.current_image_data = image_matrix
        return image_matrix
    

    def draw_start_and_goal(self, path_data : PathData, number_of_agents : int, image_matrix:np.ndarray|None = None) -> np.ndarray:
        if image_matrix is None:
            image_matrix = self.current_image_data
        color = self.generate_distinct_colors(number_of_agents, path_data.planner_id)
        image_matrix[path_data.goal[0], path_data.goal[1]] = color
        image_matrix[path_data.start[0], path_data.start[1]] = color
        self.current_image_data = image_matrix
        return image_matrix
    

    def publish_image(self, image_matrix:np.ndarray|None = None) -> None:
        if image_matrix is None:
            image_matrix = self.current_image_data
        image_cv = np.uint8(image_matrix)
        image_msg : Image = self.cv_bridge.cv2_to_imgmsg(image_cv, encoding="rgb8")
        #cv2.imshow("debug image", image_cv)
        #cv2.waitKey(0)
        self.debug_image_pub.publish(image_msg)
        return None
    

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
    

    def show_live_path(self, paths : list[PathData]) -> None:
        refresh_rate : int = 100 #hz
        rate: rospy.Rate = rospy.Rate(refresh_rate)

        elapsed_time : float = 0
        start_time : float = time.time()
        time_factor : float = 5.0 # used to speed up sim

        last_goal_timestamp : float = 0
        for path in paths:
            time_for_reaching_goal : float = path.timings[path.goal[0], path.goal[1]] 
            if time_for_reaching_goal > last_goal_timestamp:
                last_goal_timestamp = time_for_reaching_goal

        end_time : float = start_time + time_for_reaching_goal
        rospy.loginfo(f"live visualization starting, simulating {time_for_reaching_goal}s at a speed of {time_factor}...")

        while elapsed_time < time_for_reaching_goal:
            marker_array : MarkerArray = MarkerArray()
            marker_array.markers = []
            marker_pub = rospy.Publisher('visualization_markers', MarkerArray, queue_size=10, latch=True)
            

            for path in paths:
                marker: Marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "formation_builder"
                marker.id = path.planner_id
                marker.type = Marker.CUBE_LIST
                marker.action = Marker.ADD
                marker.lifetime = rospy.Duration(0, 300_000_000) #! make this smarter (?) use occupied until
                #marker.lifetime = rospy.Duration(0, int(1_000_000_000 * 20 / time_factor))

                marker.pose.orientation.w = 1.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.scale.x = 1.4 #!map scale here
                marker.scale.y = 1.4 #!map scale here
                marker.scale.z = 0.1
                marker.points = []
                path_color = self.generate_distinct_colors(len(paths), path.planner_id, value=0.7)

                marker.color.r = path_color[0]/255
                marker.color.g = path_color[1]/255
                marker.color.b = path_color[2]/255
                marker.color.a = 0.7 # alpha value for transparancy

                for index, path_point in enumerate(path.path_world):
                    x = path.path_pixel[index][0]
                    y = path.path_pixel[index][1]
                    
                    if float(np.abs(path.timings[x, y] - elapsed_time)) < 3:
                        point : Point = Point()
                        point.x = path_point[0]
                        point.y = path_point[1]
                        point.z = 0.05
                        marker.points.append(point)
                if marker.points:
                    marker_array.markers.append(marker)
            marker_pub.publish(marker_array)

            rate.sleep()
            elapsed_time = (time.time() - start_time) * time_factor
            #rospy.loginfo(f"simulated {elapsed_time}s")
        rospy.loginfo("live visualization done!")
        return None

    

fb_visualizer : Visualization = Visualization()