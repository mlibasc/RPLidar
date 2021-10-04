import pygame
import numpy as np
import sys
import cv2
from freenect import sync_get_depth as get_depth


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
    dc = (x[0] / 640) * 100
    print ('dc: ', dc)
    return dc, x[1]


if __name__ == "__main__":
    fpsClock = pygame.time.Clock()
    FPS = 30 # kinect only outputs 30 fps
    disp_size = (640, 480)
    pygame.init()
    screen = pygame.display.set_mode(disp_size)
    font = pygame.font.SysFont("Arial", 32) # provide your own font 
    while True:
        events = pygame.event.get()
        for e in events:
            if e.type == pygame.QUIT:
                sys.exit()
        fps_text = "FPS: {0:.2f}".format(fpsClock.get_fps())
        # draw the pixels

        depth = np.rot90(get_depth()[0]) # get the depth readinngs from the camera
        
        msg = pwmDir(findMaxDist(depth))
        
        pixels = gamma[depth] # the colour pixels are the depth readings overlayed onto the gamma table
        ###
        #process_data(pixels)
        #x,y = findMaxDist(depth)
        #cv2.circle(dst, (xMax,yMax), 10, (0,0,255), 10)
        #pygame.draw.circle(screen, (0,0,255), (x, y), 20)

        temp_surface = pygame.Surface(disp_size)
        pygame.surfarray.blit_array(temp_surface, pixels)
        pygame.transform.scale(temp_surface, disp_size, screen)
        screen.blit(font.render(fps_text, 1, (255, 255, 255)), (30, 30))
        pygame.display.flip()
        fpsClock.tick(FPS)
