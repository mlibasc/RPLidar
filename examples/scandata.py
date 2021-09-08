#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import RPi.GPIO as GPIO
import time

path = 18
PORT_NAME = '/dev/ttyUSB0'
DMAX = 4000
IMIN = 0
IMAX = 50

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(path, GPIO.OUT)

def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def sendDir( angle):
    
    if(angle >=0 and angle < 180): 
        GPIO.output(path, GPIO.HIGH)
        print "0<=x<180"
        time.sleep(0.1)
    else:
        GPIO.output(path, GPIO.LOW)
        print "180<=x<360"
        time.sleep(0.1) 

def run():
    lidar = RPLidar(PORT_NAME)
    #fig = plt.figure()
    #ax = plt.subplot(111, projection='polar')
    #line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX], cmap=plt.cm.Greys_r, lw=0)
   
    #xVals = [0, maxDist]
    #yVals = [0, maxAngle]
    #plt.plot(xVals,yVals)
    #ax.set_rmax(DMAX)
    #ax.grid(True)

    while(True):
        iterator = lidar.iter_scans()
        scan = next(iterator)
        while(scan):
            # Method #1
            # looping through the elements in scan to get the max distance
            #for data  in scan:
            #    if(data[2] > maxDist):
            #        maxDist = data[2]
            #        maxAngle = data[1]
            
            # Method #2
            # rolling average to find the most open space
            print(len(scan))
            maxDist = -1
            maxAngle = -1
            for i in range(len(scan)-10):
                avg = 0
                midAngle = 0
                for j in range(10):
                    avg += scan[i+j][2]
                    midAngle += scan[i+j][1]
                avg /= 10
                midAngle /= 10
                if(avg > maxDist):
                    maxDist = avg
                    maxAngle = midAngle 
                print ('average = %.2f, angle = %f' % (avg, midAngle))


            # have a running average to get the most "open space"
            # loop through the elements, have an incremental array, while looping through the array,
            # take new averages as new elements are looped through
            # at the end, output the angle/distance of the most open space            

            print(scan)
            print('maxDist: ',  maxDist, " maxAngle: ", maxAngle)
            sendDir(maxAngle)
            scan = next(iterator)
    #iterator = lidar.iter_scans()
    #ani = animation.FuncAnimation(fig, update_line,
    #    fargs=(iterator, line), interval=50)
    #plt.show()
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt: # IF CTRL+C is pressed, exit cleanly:
        print('quitting')
    finally:
        GPIO.cleanup() # Clean up GPIO
    
