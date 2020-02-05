debug = 1

if debug:
    print('importing modules')
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
#import matplotlib.pyplot as plt
from skimage import color, morphology, measure, io
#from skimage import io

def filter_pink(image):
    if debug:
        print('starting filter_pink')
        print('converting to hsv')
    hsv_image = color.rgb2hsv(image)
    if debug:
        print('evaluating pixels')
    pink = np.zeros(np.shape(hsv_image)[0:2])
    for i in range(np.shape(hsv_image)[0]):
        for j in range(np.shape(hsv_image)[1]):
            if np.abs(hsv_image[i,j,0]-0.8333) < .1:
                pink[i,j] = 1
    return(pink)

def filter_noise(image):
    if debug:
        print('starting filter_noise')
    filtered = morphology.opening(image, morphology.disk(6))
    return filtered

def label_ball(image):
    if debug:
        print('starting label_ball')
    labels = measure.label(image)
    props = measure.regionprops(labels)
    if debug:
        print(props,' objects found')
    biggest_area = 0
    maxi = 0
    for i in range(len(props)):
        width = props[i].bbox[1]-props[i].bbox[0]
        height = props[i].bbox[3]-props[i].bbox[2]
        area = width*height
        if area > biggest_area:
            biggest_area = area
            maxi = i
    if not (biggest_area == 0):
        center = np.array([int(0.5*(props[maxi].bbox[2]+props[maxi].bbox[0])),int(0.5*(props[maxi].bbox[3]+props[maxi].bbox[1]))])
    else:
        center = [320, 240]
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
loop = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	print('loop',loop)
	loop += 1
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	# if the `q` key was pressed, break from the loop
#	if key == ord("q"):
#		break

	pink = filter_pink(image)
	no_noise = filter_noise(pink)
	center = label_ball(no_noise)
	print(center)

	# clear the stream, for the next frame
	rawCapture.truncate(0)
