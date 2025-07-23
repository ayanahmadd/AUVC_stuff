import cv2
import numpy as np


def detect_lines(img, threshold1=50, threshold2=150, apertureSize=3, minLineLength=100, maxLineGap=10):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, threshold1, threshold2, apertureSize=apertureSize)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi/180,
            100,
            minLineLength = minLineLength,
            maxLineGap = maxLineGap
        )
        return lines
    
def draw_lines(img, lines, color=(0, 255, 0)):
            imgcopy = img.copy()
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(imgcopy, (x1, y1), (x2, y2), color, 2)
            return imgcopy

def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []

    if lines is not None:
        for line in lines:
            try:
                x1, y1, x2, y2 = map(float, line[0])  # float conver!!!!
                if x2 - x1 == 0:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1
                slopes.append(slope)
                intercepts.append(intercept)
            except Exception as e:
                print(f"Skipping line {line} due to error: {e}")
                continue

    return slopes, intercepts

def detect_lanes(lines, slope_threshold=0.1, intercept_threshold=50):
    slopes, intercepts = get_slopes_intercepts(lines)
    lanes = []

    for i in range(len(slopes)):
        for j in range(i + 1, len(slopes)):
            try:
                slope_diff = abs(slopes[i] - slopes[j])
                intercept_diff = abs(intercepts[i] - intercepts[j])
                if slope_diff < slope_threshold and intercept_diff < intercept_threshold:
                    lanes.append([lines[i], lines[j]])
            except Exception as e:
                print(f"Error comparing slopes at {i}, {j}: {e}")
                continue

    return lanes

def draw_lanes(img, lanes):
    img_copy = img.copy()
    colors = [(255, 0, 0), (0, 255, 255), (255, 0, 255), (0, 128, 255)]
    for idx, lane in enumerate(lanes):
        color = colors[idx % len(colors)]
        for line in lane:
            x1, y1, x2, y2 = line[0]
            cv2.line(img_copy, (x1, y1), (x2, y2), color, 2)
    return img_copy



    
    
    
   