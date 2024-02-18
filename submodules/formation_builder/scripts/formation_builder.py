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
    show_debug_images : bool = True

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
        rospy.loginfo("------------ Done ------------")
        return None
    


    def publish_image(self, topic_name : str, matrix : np.ndarray) -> None:
        img_publisher : rospy.Publisher = rospy.Publisher('/formation_builder/'+topic_name, Image, queue_size=10)
        ros_image : Image = self.cvbridge.cv2_to_imgmsg(matrix, encoding="mono8")
        img_publisher.publish(ros_image)
        rospy.loginfo(f"updated image on topic: {topic_name}...")
        return None
    
    def show_image(self, img, name: str) -> None:
        if self.show_debug_images:
            target_size = (500, 500)  # Festlegen der Zielgröße für die Bilder
            resized_img = cv2.resize(img, target_size)#, interpolation=cv2.INTER_AREA)  # Skalieren des Bildes auf die Zielgröße
            cv2.imshow(name, resized_img)
            cv2.waitKey(0)
        return None

        

if __name__ == '__main__':
    fb : FormationBuilder = FormationBuilder()
    rospy.init_node('map_reader')
    rospy.spin()
