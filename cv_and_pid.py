# import the necessary packages
import pypar as pp
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import imutils
import numpy as np

#motor control libraries
import LS7366R
import RPi.GPIO as IO


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

#setup input and output pins on Pi
    IO.setmode(IO.BCM)

    dir_1 = 15
    pwm_1_pin = 12

    IO.setup(dir_1, IO.OUT, initial = 0)

    IO.setup(pwm_1_pin, IO.OUT)
    pwm_1 = IO.PWM(pwm_1_pin, 50000) #50kHz PWM frequency

#setup Encoder
    encoder = LS7366R(0, 1000000, 4)

'''
counts_per_pix depends on camera distance to table, camera resolution,
#encoder resolution, pulley radius:

encoder resolution = 1000 counts per rotation
table track height = 480 mm
camera resolution height = 1080 pix, maps directly onto 480 mm if set properly
pulley radius = 14 mm
'''

    counts_per_pix = (1000*480)/(2*3.14159*14*1080)

#PID variables
    delta_u = 0
    u_max = 100
    u_min = -100

    #error values
    e1 = 0
    e2 = 0
    e3 = 0

    #PID gains
    kp = 2
    ki = 2
    kd = 2

    #calculate discrete time coefficients
    k1 = kp+ki+kd
    k2 = -kp-2*kd
    k3 = kd

    #start pwm_1
    dir = 0
    u = 0
    pwm_1.start(u)

#PID control loop
    while 1:
        counts = encoder.readCounter()
        target= pp.receive(source=0)
        target_counts = counts_per_pix*target

        e3 = e2
        e2 = e1
        e1 = target_counts-counts

        delta_u = k1*e1 + k2*e2 + k3*e3
        u = u+delta_u

        if u>=0 and u<=u_max:
            dir = 0
        elif u<0 and u>=u_min:
            u = -u
            dir = 1
        elif u > u_max:
            u = u_max
            dir = 0
        else:
            u = -u_min
            dir = 1

        IO.output(dir_1, dir)
        pwm_1.ChangeDutyCycle(u)

        time.sleep(.01) #PID algorithm will run at approximately 10 ms

pp.finalize() #close paralell processes
