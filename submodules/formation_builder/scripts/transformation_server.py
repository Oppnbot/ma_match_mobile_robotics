#!/usr/bin/env python3.8

from __future__ import annotations
import rospy
from formation_builder.msg import GridMap
from formation_builder.srv import TransformPixelToWorld, TransformPixelToWorldResponse, TransformPixelToWorldRequest
from formation_builder.srv import TransformWorldToPixel, TransformWorldToPixelResponse, TransformWorldToPixelRequest
import numpy as np


class TransformationServer:
    def __init__(self) -> None:
        rospy.init_node('transformation_server')
        rospy.Subscriber('/formation_builder/gridmap', GridMap, self.update_scaling_factor)
        #rospy.Service('pixel_to_world', WorldPos, self.pixel_to_world_callback)
        #rospy.Service('world_to_pixel', PixelPos, self.world_to_pixel_callback)
        rospy.Service('pixel_to_world', TransformPixelToWorld, self.pixel_to_world_callback)
        #rospy.Service('world_to_pixel', TransformWorldToPixel, self.world_to_pixel_callback)
        self.scaling_factor : float | None = None # world res / pixel res
        self.grid_size : tuple[int, int] | None = None
        rospy.spin()
        return None
    

    def pixel_to_world_callback(self, req : TransformPixelToWorldRequest) -> TransformPixelToWorldResponse:
        if self.scaling_factor is None:
            rospy.logwarn("scaling factor is None. This is likely due missing map data")
            return TransformPixelToWorldResponse([], [])
        if self.grid_size is None:
            rospy.logwarn("Grid Size is None. This is likely due missing map data")
            return TransformPixelToWorldResponse([], [])
        if len(req.x_pixel) != len(req.y_pixel):
            rospy.logwarn(f"Transformation failed! must have the same ammount of x- and y-Values. x: {len(req.x_pixel)}, y: {len(req.y_pixel)}")
            return TransformPixelToWorldResponse([], [])
        world_positions_x : list[float] = [(y_pixel + 0.5) * self.scaling_factor for y_pixel in req.y_pixel]
        world_positions_y : list[float] = [((self.grid_size[0] - x_pixel) - 0.5) * self.scaling_factor for x_pixel in req.x_pixel]
        return TransformPixelToWorldResponse(world_positions_x, world_positions_y)
    

    #def world_to_pixel_callback(self, req : TransformWorldToPixelRequest) -> TransformWorldToPixelResponse:
    #    #! this function is untested and may need some testing
    #    if self.scaling_factor is None:
    #        rospy.logwarn("scaling factor is None. This is likely due missing map data")
    #        return TransformWorldToPixelResponse(0, 0)
    #    pixel_pos_x : int = np.round(req.y_world / self.scaling_factor)
    #    pixel_pos_y : int = np.round(req.x_world / self.scaling_factor)
    #    return TransformWorldToPixelResponse(pixel_pos_x, pixel_pos_y)
    
    
    def update_scaling_factor(self, grid_map : GridMap) -> None:
        self.scaling_factor = grid_map.resolution_map / grid_map.scaling_factor
        self.grid_size = (grid_map.grid_width, grid_map.grid_height)
        return None
    


if __name__ == '__main__':
    transformation_server : TransformationServer = TransformationServer()