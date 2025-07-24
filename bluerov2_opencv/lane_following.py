import numpy as np

def get_closest_lane(lanes):
    if not lanes:
        return None

    max_slope = None
    best_lane = None

    for lane in lanes:
        x_vals = []
        y_vals = []

        for line in lane:
            x1, y1, x2, y2 = line  # assuming line = [x1, y1, x2, y2]
            x_vals.extend([x1, x2])
            y_vals.extend([y1, y2])

        if len(x_vals) < 2:
            continue  # skip this lane

        slope, intercept = np.polyfit(x_vals, y_vals, 1)

        if max_slope is None or abs(slope) > abs(max_slope):
            max_slope = slope
            best_lane = lane

    return best_lane
    

def get_lane_center(lanes):
    if not lanes:
        return None, None
    our_best_lane = get_closest_lane(lanes)  # find closest lane
    if our_best_lane is None:
        return None, None
    
    x_vals = []
    y_vals = []
    for line in our_best_lane:
        x1, y1, x2, y2 = line
        x_vals.extend([x1, x2])
        y_vals.extend([y1, y2])
    if len(x_vals) < 2:
        return None, None
    center_slope, center_intercept = np.polyfit(x_vals, y_vals, 1)
    return center_slope, center_intercept


# def get_lane_center(lanes):
#     if not lanes:
#         return None, None
#     lane = lanes[0]  # assume first lane is closest
#     x_vals = []
#     y_vals = []
#     for line in lane:
#         x1, y1, x2, y2 = line[0]
#         x_vals.extend([x1, x2])
#         y_vals.extend([y1, y2])
#     if len(x_vals) < 2:
#         return None, None
#     slope, intercept = np.polyfit(x_vals, y_vals, 1)
#     return intercept, slope

def recommend_direction(center, slope, image_width=640):
    if center is None:
        return "unknown"
    center_x = image_width / 2
    lane_x = (center - slope * (image_width // 2))
    if lane_x < center_x - 50:
        return "left"
    elif lane_x > center_x + 50:
        return "right"
    else:
        return "forward"
