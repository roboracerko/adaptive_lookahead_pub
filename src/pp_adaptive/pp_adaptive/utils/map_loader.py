#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Map loading utilities for ROS-style occupancy maps.
"""

import os
import math
from typing import Tuple

import numpy as np
import yaml
from PIL import Image


class OccupancyMap:
    """Occupancy map for collision checking."""
    
    def __init__(self, map_yaml: str, occ_thresh: int = 250):
        with open(map_yaml, "r") as f:
            y = yaml.safe_load(f)

        # resolve image path relative to yaml
        img_rel = y["image"]
        base_dir = os.path.dirname(os.path.abspath(map_yaml))
        img_path = img_rel if os.path.isabs(img_rel) else os.path.join(base_dir, img_rel)

        if not os.path.exists(img_path):
            raise FileNotFoundError(f"[ERROR] Map image not found: {img_path}")

        self.res = float(y["resolution"])
        self.ox = float(y["origin"][0])
        self.oy = float(y["origin"][1])
        self.oyaw = float(y["origin"][2])

        img = np.array(Image.open(img_path).convert("L"))
        self.H, self.W = img.shape
        self.occ = img
        self.thresh = occ_thresh

        print(f"[INFO] Map loaded: {img_path}, shape={img.shape}")

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        dx = x - self.ox
        dy = y - self.oy

        c = math.cos(-self.oyaw)
        s = math.sin(-self.oyaw)
        xr = c * dx - s * dy
        yr = s * dx + c * dy

        col = int(xr / self.res)
        row = int((self.H - 1) - (yr / self.res))
        return row, col

    def is_collision(self, x: float, y: float) -> bool:
        row, col = self.world_to_pixel(x, y)
        if row < 0 or row >= self.H or col < 0 or col >= self.W:
            return True
        return self.occ[row, col] < self.thresh
