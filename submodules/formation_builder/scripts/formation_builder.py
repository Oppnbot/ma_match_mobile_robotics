#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

from pyclbr import Class
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import random
from visualization_msgs.msg import Marker



class FormationBuilder:
    resolution : float = 1.0 # [m] per grid cell
    

    def __init__(self) -> None:
        self.input_map: OccupancyGrid | None = None
        self.occupied_from: np.ndarray | None = None
        self.occupied_until: np.ndarray | None = None

        self.cvbridge : CvBridge = CvBridge()
        #image_pub = rospy.Publisher('/my_grid_img', Image, queue_size=10)
        rospy.Subscriber('/map', OccupancyGrid, self.read_map)
        self.image_pub : rospy.Publisher = rospy.Publisher('/formation_builder/debug_img', Image, queue_size=10)
        return None
    
    def read_map(self, map_data : OccupancyGrid) -> None:
        rospy.loginfo("got new data")

        rospy.loginfo(f"NEW MAP DATA ({map_data.header.frame_id}): \nsize:\n  width:\t{map_data.info.width}\n  height:\t{map_data.info.height}\nresolution:\n  {map_data.info.resolution}\n{map_data.info.origin}")
 
        self.input_map = map_data
        self.build_occupancy_grid(map_data)
        return None

    
    def build_occupancy_grid(self, map_data : OccupancyGrid) -> None:
        scaling_factor : float = map_data.info.resolution / self.resolution

        map_array : np.ndarray = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        map_array = (255 * (1 - map_array / 100)).astype(np.uint8)
        
        #cv_image = cv2.cvtColor(map_array, cv2.COLOR_GRAY2BGR)

        grid_width : int = int(np.round(map_data.info.width * scaling_factor))
        grid_height : int = int(np.round(map_data.info.height * scaling_factor))

        scaled_image = cv2.resize(map_array, (grid_width, grid_height))

        rate = rospy.Rate(1) # we somehow need this to update an image. #! dont remove
        rate.sleep()

        rospy.loginfo(f"Resized Image to a grid size of {self.resolution}cells/m:\n  new width:\t{grid_width}\n  new height:\t{grid_height}")
        
        self.publish_image(self.image_pub, scaled_image)


        rospy.loginfo("------------ Done ------------")
        return None
    
    

    def publish_image(self, publisher : rospy.Publisher, matrix : np.ndarray) -> None:
        ros_image : Image = self.cvbridge.cv2_to_imgmsg(matrix, encoding="mono8")
        publisher.publish(ros_image)
        rospy.loginfo("updated image...")
        return None
    
        

if __name__ == '__main__':
    fb : FormationBuilder = FormationBuilder()
    rospy.init_node('formation_builder')
    rospy.spin()
