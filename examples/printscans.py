#!/usr/bin/env python3
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = '/dev/ttyUSB0'
DMAX = 4000
IMIN = 0
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def run():
    lidar = RPLidar(PORT_NAME)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                           cmap=plt.cm.Greys_r, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    while(True):
        iterator = lidar.iter_scans()
        scan = next(iterator)
        while(scan):
            # Do stuff with scan data here
            # CASE 1
            # loop through the elements in scan to get the largest number for distance
            # CASE 2
            # have a running average to get the most "open space"
            # loop through the elements, have an incremental array, while looping through the array,
            # take new averages as new elements are looped through
            # at the end, output the angle/distance of the most open space            

            print(scan)
            scan = next(iterator)
    #iterator = lidar.iter_scans()
    #ani = animation.FuncAnimation(fig, update_line,
    #    fargs=(iterator, line), interval=50)
    #plt.show()
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    run()
