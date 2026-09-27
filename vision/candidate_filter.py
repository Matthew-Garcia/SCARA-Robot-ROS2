"""Camera-independent size and image-edge checks for blue candidates."""

import math


def candidate_fits(area, rectangle, image_size, minimum_area, maximum_area):
    if not math.isfinite(area) or not 0 < minimum_area <= area <= maximum_area:
        return False
    x, y, width, height = rectangle
    image_width, image_height = image_size
    # Bounding rectangles use an exclusive right/bottom edge.
    return (width > 0 and height > 0 and x > 0 and y > 0
            and x + width < image_width and y + height < image_height)
