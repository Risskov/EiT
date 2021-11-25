import cv2
import numpy as np
import time

def show_image(image_name, image):
    cv2.imshow(image_name, image)
    #cv2.waitKey()
    time.sleep(0.1)



# Segmentation

def color_distance(colorCenter, segmentColor):
    meanrcolor = (colorCenter[2]+segmentColor[2])/2
    r = colorCenter[2]-segmentColor[2]
    g = colorCenter[1]-segmentColor[1]
    b = colorCenter[0]-segmentColor[0]
    return np.sqrt((2+meanrcolor/256)*r**2+4*g**2+(2+(255-meanrcolor)/256)*b**2)

def find_smallest_distances(centers, color_value):
    distances = []
    for center in centers:
        distances.append(abs(center[0] - color_value))#color_distance(center[0:3], color_value))
    return sorted(range(len(distances)), key=lambda k: distances[k])