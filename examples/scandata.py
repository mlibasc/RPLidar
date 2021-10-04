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
    depth[depth == 2047] = 0
    dst = cv2.GaussianBlur(depth, (11,11), 0)
    val = np.amax(dst)

    #print("Max X Coord: ", int(np.mean(np.where(dst == val)[0])))
    #print("var X Coord: ", int(np.sqrt(np.var(np.where(dst > (val-50))[0]))))

    #print("val at that place", depth[int(np.mean(np.where(dst == val)[0]))][np.where(dst == val)[1][0]])

    return int(np.mean(np.where(dst==val)[0])),int(np.sqrt(np.var(np.where(dst == val)[0])))

def pwmDir(x):
    # 58.5 x 46.6 degrees field of view
    # about 5 x 5 pixels per degree
    # when heading is the same as maxAngle, dc = 50%
    # 640 x 480 pixels
    
    # kinect will only send out ~42 pwm to 58 pwm
    # left is greater than 50, right is less than 50
    # convert std of index to angle
    #print('kinect std: ', x[1])
    kinCert = x[1]/640 * 58.5
    dc = (x[0] / 640) * 17/2 + 50
    #print ('kinect dc: ', dc)
    #print ('kin Cert: ', kinCert)
    return dc, kinCert

def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def led(maxAngle, rollAvg, maxDist, kinpwm, findMaxDist):
    # when heading is the same as maxAngle, dc = 50%
    #cerfact = std/size of array * 360
    lidstd = int(np.sqrt(np.var(np.where(rollAvg > (maxDist - 50)))))
    lidCert = lidstd/len(rollAvg) * 360
    liddc = (maxAngle / 360) * 100

    kinstd = findMaxDist[1]
    kinCert = kinpwm[1]
    kindc = kinpwm[0]

    print('kin std: ', kinstd)
    print ('kin Cert: ', kinCert)
    print ('kin dc: ', kindc)
    
    print('lid std: ', lidstd)
    print ('lidar Cert: ', lidCert)
    print ('lidar dc: ', liddc)

    if(kinCert*10 > lidCert):
        pwm.ChangeDutyCycle(kindc)
        print('output kindc: ', kindc)
    else:
        pwm.ChangeDutyCycle(liddc)
        print('output liddc: ', liddc)
    
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
            rollAvg = np.array([])
            
            # average the first 10 samples 
            for i in range(increment):
                avg += scan[i][2]
                avgAngle += scan[i][1]
            avg /= increment
            avgAngle /= increment

            rollAvg = np.append(rollAvg, avg)

            for i in range(increment, len(scan)):
                avg = ((avg * multA) + (scan[i][2] * multB)) / (multA + multB)
                avgAngle = ((avgAngle * multA) + (scan[i][1] * multB)) / (multA + multB)
                rollAvg = np.append(rollAvg, avg)
                if(avg > maxDist):
                    maxDist = avg
                    maxAngle = avgAngle
                if(avg < minDist):
                    minAngle = avgAngle

            depth = np.rot90(get_depth()[0]) # get the depth readinngs from the camera
            #pwmDir(findMaxDist(depth))
            print('maxDist: ',  maxDist, " maxAngle: ", maxAngle)
            print('minDist: ', minDist, " minAngle: ", minAngle)
            led(maxAngle, rollAvg, maxDist, pwmDir(findMaxDist(depth)), findMaxDist(depth))
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
    
