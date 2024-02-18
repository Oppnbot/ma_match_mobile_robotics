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



class MapReader:
    resolution : float = 1.0 # [m] per grid cell
    show_debug_images : bool = False
    show_debug_prints : bool = True

    def __init__(self) -> None:
        self.input_map: OccupancyGrid | None = None
        self.cvbridge : CvBridge = CvBridge()
        rospy.init_node('map_reader')
        rospy.Subscriber('/map', OccupancyGrid, self.read_map)
        self.image_pub : rospy.Publisher = rospy.Publisher('/formation_builder/map', Image, queue_size=10, latch=True)
        rospy.spin()
        return None
    

    def read_map(self, map_data : OccupancyGrid) -> None:
        if self.show_debug_prints:
            rospy.loginfo("got new data")
            rospy.loginfo(f"NEW MAP DATA ({map_data.header.frame_id}): \nsize:\n  width:\t{map_data.info.width}\n  height:\t{map_data.info.height}\nresolution:\n  {map_data.info.resolution}\n{map_data.info.origin}")
 
        self.input_map = map_data

        scaling_factor : float = map_data.info.resolution / self.resolution
        if self.show_debug_prints:
            rospy.loginfo(f"Scaling factor: {scaling_factor}")

        map_array : np.ndarray = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        map_array = (255 * (1 - map_array / 100)).astype(np.uint8)
        
        #cv_image = cv2.cvtColor(map_array, cv2.COLOR_GRAY2BGR)

        #* BINARIZE
        # Apply Threshhold to enable usage of morph operators etc
        ret, orig_img_thresh = cv2.threshold(map_array, 127, 255, cv2.THRESH_BINARY)
        self.show_image(orig_img_thresh, "Orig Thresh")

        #* CLOSING 
        # Closing for denoising with a small kernel
        kernel_size : int = 1
        kernel : np.ndarray = np.ones((kernel_size, kernel_size))
        denoised_image = cv2.morphologyEx(orig_img_thresh, cv2.MORPH_CLOSE, kernel)
        self.show_image(denoised_image, "Denoised")
        
        #* ERODATION
        # we have to increase the size of the walls for them to remain after downscaling
        kernel_size : int = int(np.round(1.0/scaling_factor))
        kernel : np.ndarray = np.ones((kernel_size, kernel_size))
        eroded_image = cv2.erode(denoised_image, kernel)
        self.show_image(eroded_image, "Eroded")

        #* SCALING
        # Downscaling so that we get a useful gridsize for path planning. cell should be robot size
        grid_width : int = int(np.round(map_data.info.width * scaling_factor))
        grid_height : int = int(np.round(map_data.info.height * scaling_factor))
        scaled_image = cv2.resize(eroded_image, (grid_width, grid_height), interpolation=cv2.INTER_NEAREST)
        self.show_image(scaled_image, "Scaled")

        #* BINARIZE
        # The scaling might result in some blurring effects, binarizing removes those
        ret, thresh_image = cv2.threshold(scaled_image, 127, 255, cv2.THRESH_BINARY)
        self.show_image(thresh_image, "Binarized")
        if self.show_debug_prints:
            rospy.loginfo(f"Resized Image to a grid size of {self.resolution}cells/m:\n  new width:\t{grid_width}\n  new height:\t{grid_height}")
        
        self.publish_image("map", thresh_image)
        return None
    


    def publish_image(self, topic_name : str, matrix : np.ndarray) -> None:
        ros_image : Image = self.cvbridge.cv2_to_imgmsg(matrix, encoding="mono8")
        self.image_pub.publish(ros_image)
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
    map_reader : MapReader = MapReader()
    #rospy.init_node('map_reader')
    #rospy.spin()
