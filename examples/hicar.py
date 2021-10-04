#!/usr/bin/env python3
'''Animates distances and measurment quality'''
# Import necessary libraries for RPLidar
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import RPi.GPIO as GPIO
import time

# Import necessary libraries for Kinect
import pygame
import sys
from freenect import sync_get_depth as get_depth

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
    pwm.ChangeDutyCycle(dc)
    print ('dc: ', dc)
    
def obstacle(obDist, obAngle):
    if(obAngle <= 180):
        print ("obstacle to right")
    else:
        print ("obstacle to left")
     
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

def run():
    lidar = RPLidar(PORT_NAME)
    while(True):
        
        # Kinect processing 
        events = pygame.event.get()
        for e in events:
            if e.type == pygame.QUIT:
                sys.exit()
        fps_text = "FPS: {0:.2f}".format(fpsClock.get_fps())
        # draw the pixels

        depth = np.rot90(get_depth()[0]) # get the depth readinngs from the camera
        pixels = gamma[depth] # the colour pixels are the depth readings overlayed onto the gamma table
        temp_surface = pygame.Surface(disp_size)
        pygame.surfarray.blit_array(temp_surface, pixels)
        pygame.transform.scale(temp_surface, disp_size, screen)
        screen.blit(font.render(fps_text, 1, (255, 255, 255)), (30, 30))
        pygame.display.flip()
        fpsClock.tick(FPS)
        
        # Lidar processing
        iterator = lidar.iter_scans()
        scan = next(iterator)
        # clockwise rotation
        while(scan):

            # rolling average to find the most open space
            #print(len(scan))
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


            #print(scan)
            print('maxDist: ',  maxDist, " maxAngle: ", maxAngle)
            print('minDist: ', minDist, " minAngle: ", minAngle)
            led(maxAngle)
            #obstacle(minDist, minAngle)
            scan = next(iterator)

    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    
    try:    
        # setup pwm
        dc = 50
        pwm.start(dc)

        # Kinect setup
        fpsClock = pygame.time.Clock()
        FPS = 30 # kinect only outputs 30 fps
        disp_size = (640, 480)
        pygame.init()
        screen = pygame.display.set_mode(disp_size)
        font = pygame.font.SysFont("Arial", 32) # provide your own font 

        run()
    except KeyboardInterrupt: # IF CTRL+C is pressed, exit cleanly:
        print('quitting')
    finally:
        GPIO.cleanup() # Clean up GPIO
    
