# ogm_cbf_kinematic_sim/utils.py

def world_to_pixel(x, y, resolution = 0.05, origin_x=0.0, origin_y=0.0, img_height=400, continuous=False):
    """
    Convert world coordinates (meters) -> image pixel coordinates.

    Uses the same convention you already use everywhere:
        pixel_x = (x - origin_x) / res
        pixel_y = img_height - ( (y - origin_y) / res )
    """
    px = (x - origin_x) / resolution
    py = img_height - ((y - origin_y) / resolution)
    if continuous:
        return float(px), float(py)
    return int(px), int(py)



def pixel_to_world(px, py, resolution = 0.05, origin_x=0, origin_y=0, img_height=400):
    """
    Convert image pixel coordinates -> world coordinates (meters),
    exact inverse of world_to_pixel().
    """
    x = px * resolution + origin_x
    y = (img_height - py) * resolution + origin_y
    return float(x), float(y)
