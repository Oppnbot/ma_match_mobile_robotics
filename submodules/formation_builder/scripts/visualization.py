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
from commons import TrajectoryData, Waypoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time
from formation_builder.msg import GridMap
from nav_msgs.msg import OccupancyGrid, GridCells


class Visualization():
    def __init__(self) -> None:
        self.cv_bridge : CvBridge = CvBridge()
        self.debug_image_pub : rospy.Publisher = rospy.Publisher("/formation_builder/debug_image", Image, queue_size=1, latch=True)
        self.grid_map_sub : rospy.Subscriber = rospy.Subscriber('/formation_builder/gridmap', GridMap, self.grid_map_callback)
        self.map_sub: rospy.Subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.grid_map : GridMap | None = None
        self.map : OccupancyGrid | None = None
        self.current_image_data : np.ndarray = np.zeros((100, 100))
        return None
    

    def grid_map_callback(self, grid_map : GridMap) -> None:
        self.grid_map = grid_map
        return None
    
    def map_callback(self, map : OccupancyGrid) -> None:
        self.map = map
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


    def draw_path(self, trajectory_data : TrajectoryData, number_of_agents : int, image_matrix:np.ndarray|None = None) -> np.ndarray:
        if image_matrix is None:
            image_matrix = self.current_image_data
        color = self.generate_distinct_colors(number_of_agents, trajectory_data.planner_id, value=0.7)
        for waypoint in trajectory_data.waypoints:
            current_val = image_matrix[waypoint.pixel_pos[0], waypoint.pixel_pos[1]]
            if all(x == 0 or x == 255 for x in current_val):
                image_matrix[waypoint.pixel_pos[0], waypoint.pixel_pos[1]] = color
            else:
                r,g,b = color
                new_color = (current_val[0] + r//2, current_val[1] + g//2, current_val[2] + b//2)
                image_matrix[waypoint.pixel_pos[0], waypoint.pixel_pos[1]] = new_color
        self.current_image_data = image_matrix
        return image_matrix
    

    def draw_start_and_goal(self, trajectory_data : TrajectoryData, number_of_agents : int, image_matrix:np.ndarray|None = None) -> np.ndarray:
        if image_matrix is None:
            image_matrix = self.current_image_data
        color = self.generate_distinct_colors(number_of_agents, trajectory_data.planner_id)
        image_matrix[trajectory_data.goal.pixel_pos[0], trajectory_data.goal.pixel_pos[1]] = color
        image_matrix[trajectory_data.start.pixel_pos[0], trajectory_data.start.pixel_pos[1]] = color
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
    



    
    def draw_timings(self, grid: np.ndarray, obstacles: np.ndarray, start: tuple[int, int], goal: tuple[int, int], waypoints: list[Waypoint] = [], dynamic_obstacles: list[TrajectoryData] = [], sleep : int | None = None) -> None:
        min_val = np.min(grid)
        max_val = np.max(grid)

        image_matrix = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)

        # Map non-visited spaces to white
        non_visited_indices = grid == -1
        image_matrix[obstacles != 0] = [255, 255, 255]

        # Map visited spaces with timing information to red/blue
        visited_indices = ~non_visited_indices
        blue_value = ((grid[visited_indices] - min_val) / (max_val - min_val) * 200 + 55).astype(np.uint8)
        image_matrix[visited_indices] = np.column_stack(((255 - blue_value), np.zeros_like(blue_value), blue_value))

        # dynamic obstacles:
        for dynamic_obstacle in dynamic_obstacles:
            for waypoint in dynamic_obstacle.waypoints:
                if waypoint.occupied_from < max_val < waypoint.occupied_until:
                    image_matrix[waypoint.pixel_pos[0], waypoint.pixel_pos[1]] = [125, 125, 125]

        # path:
        for waypoint in waypoints:
            image_matrix[waypoint.pixel_pos[0], waypoint.pixel_pos[1]] = [0, 125 , 0]
        
        # Plot start and goal points
        image_matrix[goal[0], goal[1]] = [255, 0, 0]
        image_matrix[start[0], start[1]] = [0, 255, 0]

        # Publish the image
        image_msg = self.cv_bridge.cv2_to_imgmsg(image_matrix, encoding="rgb8")
        timing_pub = rospy.Publisher("/timing_image", Image, queue_size=20, latch=True)
        timing_pub.publish(image_msg)

        if sleep is not None:
            rate = rospy.Rate(sleep)
            rate.sleep()
        return None
    


    def show_live_path(self, trajectories : list[TrajectoryData]) -> None:
        if trajectories is None:
            rospy.logwarn("Can't visualize live path, since there are no trajectories to visualize!")
            return None
        refresh_rate : int = 50 #hz
        rate: rospy.Rate = rospy.Rate(refresh_rate)

        elapsed_time : float = 0
        start_time : float = time.time()
        time_factor : float = 10.0 # used to speed up sim

        last_goal_timestamp : float = 0.0
        for trajectory in trajectories:
            if trajectory.goal.previousWaypoint is None:
                continue
            time_for_reaching_goal : float = trajectory.goal.previousWaypoint.occupied_until + 1
            if time_for_reaching_goal < float('inf') and time_for_reaching_goal > last_goal_timestamp:
                last_goal_timestamp = time_for_reaching_goal

        if last_goal_timestamp == float('inf') or last_goal_timestamp <= 0:
            rospy.logwarn(f"Trying to simulate for {last_goal_timestamp} ammount of time; will stop after 120s instead.")
            last_goal_timestamp = 120

        end_time : float = start_time + time_for_reaching_goal
        rospy.loginfo(f"live visualization starting, simulating {time_for_reaching_goal}s at a speed of {time_factor}...")

        while elapsed_time < time_for_reaching_goal:
            marker_array : MarkerArray = MarkerArray()
            marker_array.markers = []
            marker_pub = rospy.Publisher('visualization_markers', MarkerArray, queue_size=10, latch=True)
            

            for trajectory in trajectories:
                def create_marker() -> Marker:
                    marker: Marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "formation_builder"
                    marker.id = trajectory.planner_id
                    marker.type = Marker.CUBE_LIST
                    marker.action = Marker.ADD
                    marker.lifetime = rospy.Duration(0, int(1_000_000_000 / time_factor))

                    marker.pose.orientation.w = 1.0
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.scale.x = 1.4 #!map scale here
                    marker.scale.y = 1.4 #!map scale here
                    marker.scale.z = 0.1
                    marker.points = []
                    path_color = self.generate_distinct_colors(len(trajectories), trajectory.planner_id, value=0.7)
                    marker.color.r = path_color[0]/255
                    marker.color.g = path_color[1]/255
                    marker.color.b = path_color[2]/255
                    marker.color.a = 0.7 # alpha value for transparancy
                    return marker
                
                robot_marker: Marker = create_marker()
                path_marker: Marker = create_marker()
                path_marker.id += 100_000
                path_marker.color.a = 0.3
                robot_marker.color.a = 0.7
                robot_marker.points = []
                path_marker.points = []

                for waypoint in trajectory.waypoints:
                    if waypoint.world_pos is None:
                        continue
                    point : Point = Point()
                    point.x = waypoint.world_pos[0]
                    point.y = waypoint.world_pos[1]
                    point.z = 0.05
                    if waypoint.occupied_from < elapsed_time < waypoint.occupied_until:
                        robot_marker.points.append(point)
                    else:
                        path_marker.points.append(point)

                if robot_marker.points:
                    marker_array.markers.append(robot_marker)
                if path_marker.points:
                    marker_array.markers.append(path_marker)
            marker_pub.publish(marker_array)

            rate.sleep()
            elapsed_time = (time.time() - start_time) * time_factor
            #rospy.loginfo(f"simulated {elapsed_time}s")
        rospy.loginfo("live visualization done!")
        return None

    

fb_visualizer : Visualization = Visualization()