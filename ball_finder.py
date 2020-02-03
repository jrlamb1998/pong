import numpy as np
import matplotlib.pyplot as plt
from skimage import io, color, morphology, measure

def filter_pink(image):
    hsv_image = color.rgb2hsv(image)
    pink = np.zeros(np.shape(hsv_image)[0:2])
    for i in range(np.shape(hsv_image)[0]):
        for j in range(np.shape(hsv_image)[1]):
            if np.abs(hsv_image[i,j,0]-0.8333) < .1:
                pink[i,j] = 1
    return(pink)

def filter_noise(image):
    filtered = morphology.opening(image, morphology.disk(6))
    return filtered

def label_ball(image):
    labels = measure.label(image)
    props = measure.regionprops(labels)
    biggest_area = 0
    maxi = 0
    for i in range(len(props)):
        width = props[i].bbox[1]-props[i].bbox[0]
        height = props[i].bbox[3]-props[i].bbox[2]
        area = width*height
        if area > biggest_area:
            biggest_area = area
            maxi = i
    center = np.array([int(0.5*(props[maxi].bbox[2]+props[i].bbox[0])),int(0.5*(props[maxi].bbox[3]+props[i].bbox[1]))])
    return center
        
image = io.imread('stock_ball.jpeg')
pink = filter_pink(image)
no_noise = filter_noise(pink)
center = label_ball(no_noise)