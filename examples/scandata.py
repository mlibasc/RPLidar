#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import RPi.GPIO as GPIO
import time

import pygame
from freenect import sync_get_depth as get_depth
import numpy as np
import sys
import cv2

#setup for importing data from kinect

path = 12
PORT_NAME = '/dev/ttyUSB0'
DMAX = 4000
IMIN = 0
IMAX = 50

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(path, GPIO.OUT)
pwm = GPIO.PWM(path, 50)
increment = 5

def make_gamma():
    """
    Create a gamma table
    """
    num_pix =2048 # there's 2048 different possible depth values
    npf = float(num_pix)
    _gamma = np.empty((num_pix, 3), dtype=np.uint16)
    for i in range(num_pix):
        v = i / npf
        v = pow(v, 3) * 6
        pval = int(v * 6 * 256)
        lb = pval & 0xff
        pval >>= 8
        if pval == 0:
            a = np.array([255, 255 - lb, 255 - lb], dtype=np.uint8)
        elif pval == 1:
            a = np.array([255, lb, 0], dtype=np.uint8)
        elif pval == 2:
            a = np.array([255 - lb, lb, 0], dtype=np.uint8)
        elif pval == 3:
            a = np.array([255 - lb, 255, 0], dtype=np.uint8)
        elif pval == 4:
            a = np.array([0, 255 - lb, 255], dtype=np.uint8)
        elif pval == 5:
            a = np.array([0, 0, 255 - lb], dtype=np.uint8)
        else:
            a = np.array([0, 0, 0], dtype=np.uint8)

        _gamma[i] = a
    return _gamma


gamma = make_gamma()

def findMaxDist(depth):
    #dst = np.dstack((depth,depth,depth)).astype(np.uint8)
    depth[depth == 2047] = 0
    dst = cv2.GaussianBlur(depth, (11,11), 0)
    val = np.amax(dst)

    #xMax = np.amaxnp.argmax(dst, axis=0)
    #yMax = np.argmax(dst, axis=1)
    
    #print("max val", val)

    print("Max X Coord: ", int(np.mean(np.where(dst == val)[0])))
    print("var X Coord: ", int(np.sqrt(np.var(np.where(dst == val)[0]))))

    #print("val at that place", depth[int(np.mean(np.where(dst == val)[0]))][np.where(dst == val)[1][0]])

    # Record both x and y coorindates for max distance
    # Draw circle over the coordinates later
    # Return only xMax for now
    return int(np.mean(np.where(dst==val)[0])),int(np.sqrt(np.var(np.where(dst == val)[0])))

def pwmDir(x):
    # 58.5 x 46.6 degrees field of view
    # about 5 x 5 pixels per degree
    # when heading is the same as maxAngle, dc = 50%
    # 640 x 480 pixels
    
    #left is greater than 50, right is less than 50
    dc = (x[0] / 640) * 17/2 + 50
    print ('dc: ', dc)
    return dc, x[1]
def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def led(maxAngle):
    # when heading is the same as maxAngle, dc = 50%
    
    dc = (maxAngle / 360) * 100
    
   # kinDc = glview.pwmDir[0]
   # print('kinect dc: ', kinDc)
    print ('lidar dc: ', dc)
    pwm.ChangeDutyCycle(dc)
    
def obstacle(obDist, obAngle):
    if(obAngle <= 180):
        print ("obstacle to right")
    else:
        print ("obstacle to left")
    
def run():
    lidar = RPLidar(PORT_NAME)
    while(True):
        iterator = lidar.iter_scans(max_buf_meas=4000)
        scan = next(iterator)
        # clockwise rotation
        while(scan):
            # rolling average to find the most open space
            maxDist = -1
            maxAngle = -1
            minDist = 500
            minAngle = 500
            avg = 0
            avgAngle = 0
            multA = 10 
            multB = 1
            
            # average the first 10 samples 
            for i in range(increment):
                avg += scan[i][2]
                avgAngle += scan[i][1]
            avg /= increment
            avgAngle /= increment

            for i in range(increment, len(scan)):
                avg = ((avg * multA) + (scan[i][2] * multB)) / (multA + multB)
                avgAngle = ((avgAngle * multA) + (scan[i][1] * multB)) / (multA + multB)
                if(avg > maxDist):
                    maxDist = avg
                    maxAngle = avgAngle
                if(avg < minDist):
                    minAngle = avgAngle

            depth = np.rot90(get_depth()[0]) # get the depth readinngs from the camera
            print("mypwmmm",pwmDir(findMaxDist(depth)))
            print('maxDist: ',  maxDist, " maxAngle: ", maxAngle)
            print('minDist: ', minDist, " minAngle: ", minAngle)
            led(maxAngle)
            #obstacle(minDist, minAngle)
            scan = next(iterator)
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    try:    
        dc = 50
        pwm.start(dc)
        run()
    except KeyboardInterrupt: # IF CTRL+C is pressed, exit cleanly:
        print('quitting')
    finally:
        GPIO.cleanup() # Clean up GPIO
    
