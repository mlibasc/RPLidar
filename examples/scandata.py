#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import RPi.GPIO as GPIO
import time

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

def sendDir(angle):
    if(angle >=0 and angle < 180): 
        GPIO.output(path, GPIO.HIGH)
        print "0<=x<180"
        time.sleep(0.1)
    else:
        GPIO.output(path, GPIO.LOW)
        print "180<=x<360"
        time.sleep(0.1)

def led(maxAngle):
    # when heading is the same as maxAngle, dc = 50%
    dc = (maxAngle / 360) * 100
    print(dc)
    pwm.ChangeDutyCycle(dc)
    print ('dc: ', dc)
     

def run():
    lidar = RPLidar(PORT_NAME)
    blind1 = 1
    blind2 = 359
    while(True):
        iterator = lidar.iter_scans()
        scan = next(iterator)
        # clockwise rotation
        while(scan):
            # apply an infinite filter 
            # get the average of 10 points, apply a factor 
            # get the next data point, apply a factor
            # add them together to get the new average

            # rolling average to find the most open space
            #print(len(scan))
            maxDist = -1
            maxAngle = -1
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

            for i in range(len(scan)):
                avg = ((avg * multA) + (scan[i][2] * multB)) / (multA + multB)
                avgAngle = ((avgAngle * multA) + (scan[i][1] * multB)) / (multA + multB)
                if(avg > maxDist):
                    maxDist = avg
                    maxAngle = avgAngle




            '''
            for i in range(len(scan)-10):
                avg = 0
                midAngle = 0
                for j in range(10):
                    avg += scan[i+j][2]
                    midAngle += scan[i+j][1]
                avg /= 10
                midAngle /= 10
                print(avg)
                if(avg > maxDist and midAngle > blind1 and midAngle < blind2):
                    maxDist = avg
                    maxAngle = midAngle 
                print ('average = %.2f, angle = %f' % (avg, midAngle))
            '''
            #print(scan)
            print('maxDist: ',  maxDist, " maxAngle: ", maxAngle)
            led(maxAngle)
            #sendDir(maxAngle)
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
    
