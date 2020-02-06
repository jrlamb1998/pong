# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import imutils
import numpy as np

pinkLower = (160,50,10)
pinkUpper = (250,255,255)

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
    ################################ FRAME PROCESSING ################
    
#    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#    print(hsv[:,:,0])
#    cv2.imwrite('just_h.png',hsv[:,:,0])
#    cv2.imwrite('just_s.png',hsv[:,:,1])
#    cv2.imwrite('just_v.png',hsv[:,:,2])
    # construct a mask for the color "pink", then perform a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, pinkLower, pinkUpper)
#    cv2.imwrite('pink_filter.png',mask)
    mask = cv2.erode(mask, None, iterations=1)
#    cv2.imwrite('erode_filter.png',mask)
    mask = cv2.dilate(mask, None, iterations=1)
#    cv2.imwrite('dilate_filter.png',mask)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    print(center)
    
    ############################ END PROCESSING #####################
    
    
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
