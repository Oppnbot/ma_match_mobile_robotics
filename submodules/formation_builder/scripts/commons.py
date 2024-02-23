#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import numpy as np
import rospy
from formation_builder.msg import GridMap

class PathData():
    def __init__(self, planner_id:int, timings:np.ndarray, path:list[tuple[int, int]] = []):
        self.planner_id : int = planner_id
        self.path_pixel : list[tuple[int, int]] = path          # path on the downscaled map [px]
        self.path_world : list[tuple[float, float]] = []        # path in world coordinates [m]
        self.occupation_list : list[tuple[float, float]] = []   # occupied [from, to]
        self.timings : np.ndarray = timings
        self.start: tuple[int, int]
        self.goal: tuple[int, int]
        if len(path) > 0:
            self.start = path[-1]
            self.goal = path[0]




