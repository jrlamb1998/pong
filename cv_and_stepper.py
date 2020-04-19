# import the necessary packages
import pypar as pp
import pigpio
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import imutils
import numpy as np


rank = pp.rank() #figure out what loop to run

########### BALL TRACKING LOOP ##################

if rank == 0:
    # define initial positions for ball tracking
    [x_old,y_old] = [0,0]
    [x_new,y_new] = [0,0]
    [dx,dy]= [0,0]

    # define color range for the ball
    pinkLower = (142,100,80)
    pinkUpper = (175,255,255)

    debug = 0  #debug means save all intermediate outputs
    center = None

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    i = 0
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
    #    image = cv2.resize(image, (320,480), interpolation = cv2.INTER_AREA)
        if debug:
            cv2.imwrite('color_pic.png',image)
        ################################ FRAME PROCESSING ################
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if debug:
            cv2.imwrite('just_h.png',hsv[:,:,0])
            cv2.imwrite('just_s.png',hsv[:,:,1])
            cv2.imwrite('just_v.png',hsv[:,:,2])
        # construct a mask for the color "pink", then perform a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, pinkLower, pinkUpper)
        if debug:
            cv2.imwrite('pink_filter.png',mask)
    #    mask = cv2.erode(mask, None, iterations=1)
    #    cv2.imwrite('erode_filter.png',mask)
    #    mask = cv2.dilate(mask, None, iterations=1)
    #    cv2.imwrite('dilate_filter.png',mask)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            except:
                pass
        #print(i,' ',center)
        i+=1
        ############################ END PROCESSING #####################
        ############ BALL PREDICTION #########################
        grid = np.zeros([640,480])
        [x_old,y_old] = [x_new,y_new]
        [x_new,y_new]= center
        [dx,dy] = [x_new-x_old,y_new-y_old]

        while (x_new <640) and (x_new >0) and (dx*dy != 0):
            if y_new < 0:
                y_new = -y_new
                dy = -dy
            if y_new > 99:
                y_new = 99-(y_new-99)
                dy = -dy
            grid[x_new,y_new] = 1
            [x_old,y_old] = [x_new,y_new]
            [x_new,y_new] = [x_old+dx,y_old+dy]
        print(y_new)
        pp.send(y_new, destination=1) #send target to stepper process


        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

###################STEPPER MOTOR LOOP#############
if rank == 1:

    #setup raspberry pi GPIO pins
    stepPin = 5
    dirPin = 2
    enPin = 8

    pi = pigpio.pi()
    pi.set_mode(stepPin, pigpio.OUTPUT)
    pi.set_mode(dirPin, pigpio.OUTPUT)
    pi.set_mode(enPin, pigpio.OUTPUT)

    position = 0
    speed = 1000
    while 1:
        target = pp.receive(source=0) #recieve target from stepper process
        if position > target:
            pi.write(dirPin,1)
            pi.write(enPin,0)
            pi.write(stepPin,1)
            time.sleep(0.5/speed)
            pi.write(stepPin,0)
            time.sleep(0.5/speed)

            position += 1
        if position < target:
            pi.write(dirPin,0)
            pi.write(enPin,0)
            pi.write(stepPin,1)
            time.sleep(0.5/speed)
            pi.write(stepPin,0)
            position -= 1
        else:
            time.sleep(1/speed)

pp.finalize() #close paralell processes
