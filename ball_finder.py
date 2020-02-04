from picamera.array import PiRGBArray
from picamera import PiCamera
import time
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


#### code taken from https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/ ####
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

	pink = filter_pink(image)
	no_noise = filter_noise(pink)
	center = label_ball(no_noise)

	# clear the stream, for the next frame
	rawCapture.truncate(0)
