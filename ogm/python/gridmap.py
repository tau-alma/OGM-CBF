"""
Class defination of the gridmap
"""

import numpy as np
from utils import bresenham, flood_fill
EXTEND_AREA = 1.0

class GridMap():
    def __init__(self, resolution=0.5, grid_size=None) -> None:
        self.occupancy_map = None
        self.xy_resolution = resolution
        

    def calc_grid_map_config(self, ox, oy, xy_resolution):
        """
        Calculates the size, and the maximum distances according to the the
        measurement center
        """
        min_x = round(min(ox) - EXTEND_AREA / 2.0)
        min_y = round(min(oy) - EXTEND_AREA / 2.0)
        max_x = round(max(ox) + EXTEND_AREA / 2.0)
        max_y = round(max(oy) + EXTEND_AREA / 2.0)
        xw = int(round((max_x - min_x) / xy_resolution))
        yw = int(round((max_y - min_y) / xy_resolution))
        print("The grid map is ", xw, "x", yw, ".")
        return min_x, min_y, max_x, max_y, xw, yw

    def extend_map(self, map, transform = [0, 0, 0]):
        self.occupancy_map = map

    def generate_ray_casting_grid_map(self, cloud: np.array, breshen=True):
        """
        The breshen boolean tells if it's computed with bresenham ray casting
        (True) or with flood fill (False)
        """
        ox = cloud[:, 0]
        oy = cloud[:, 1]
        min_x, min_y, max_x, max_y, x_w, y_w = self.calc_grid_map_config(ox, oy, self.xy_resolution)
        center_x = int(
            round(-min_x / self.xy_resolution))  # center x coordinate of the grid map
        center_y = int(
            round(-min_y / self.xy_resolution))  # center y coordinate of the grid map
        # occupancy grid computed with bresenham ray casting
        occupancy_map = np.ones((x_w, y_w)) / 2
        if breshen:
            for (x, y) in zip(ox, oy):
                # x coordinate of the the occupied area
                ix = int(round((x - min_x) / self.xy_resolution))
                # y coordinate of the the occupied area
                iy = int(round((y - min_y) / self.xy_resolution))
                laser_beams = bresenham((center_x, center_y), (ix, iy))  # line form the lidar to the occupied point
                for laser_beam in laser_beams:
                    occupancy_map[laser_beam[0]][laser_beam[1]] = 0.0  # free area 0.0
                occupancy_map[ix][iy] = 1.0  # occupied area 1.0
                occupancy_map[ix][iy-1] = 1.0  # extend the occupied area
                occupancy_map[ix-1][iy] = 1.0  # extend the occupied area
                occupancy_map[ix][iy] = 1.0  # extend the occupied area # extend the occupied area
        # occupancy grid computed with with flood fill
        else:
            flood_fill((center_x, center_y), occupancy_map)
            occupancy_map = np.array(occupancy_map, dtype=float)
            for (x, y) in zip(ox, oy):
                ix = int(round((x - min_x) / self.xy_resolution))
                iy = int(round((y - min_y) / self.xy_resolution))
                occupancy_map[ix][iy] = 1.0  # occupied area 1.0
                occupancy_map[ix][iy-1] = 1.0  # extend the occupied area
                occupancy_map[ix-1][iy] = 1.0  # extend the occupied area
                occupancy_map[ix][iy] = 1.0  # extend the occupied area

        self.extend_map(occupancy_map)
        return self.occupancy_map, min_x, max_x, min_y, max_y, self.xy_resolution