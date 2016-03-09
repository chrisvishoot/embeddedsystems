#!/usr/bin/env python

'''
Track a orange ball using OpenCV.

    Copyright (C) 2015 Conan Zhao and Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License 
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import cv2
import numpy as np
import time
import mraa

# For OpenCV2 image display
WINDOW_NAME = 'OrangeBallTracker' 

myServo0 = mraa.Pwm(3)
myServo0.period_ms(19)
myServo0.enable(True)

def track(image):

    '''Accepts BGR image as Numpy array
       Returns: (x,y) coordinates of centroid if found
                (-1,-1) if no centroid was found
                None if user hit ESC
    '''

    # Blur the image to reduce noise
    blur = cv2.GaussianBlur(image, (5,5),0)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image for only orange colors
    lower_orange = np.array([11,  180, 80]) # 7.40, 0.571, 0.504
    upper_orange = np.array([15, 255, 255]) # 10, 0.795, 0.415

    # Threshold the HSV image to get only orange colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5,5),0)

    # Take the moments to get the centroid
    moments = cv2.moments(bmask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)

    # Assume no centroid
    ctr = (-1,-1)

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:

        ctr = (centroid_x, centroid_y)

        # Put black circle in at centroid in image
        cv2.circle(image, ctr, 4, (0,0,0))

    # Display full-color image
    #cv2.imshow(WINDOW_NAME, image)
    print ctr

    # Force image display, setting centroid to None on ESC key input
    if cv2.waitKey(1) & 0xFF == 27:
        ctr = None
    
    # Return coordinates of centroid
    return ctr

in_min =   0
in_max = 140
out_min =  0.0199
out_max =  0.091

def mapValue(value):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Test with input from camera
if __name__ == '__main__':

    capture = cv2.VideoCapture(0)
    capture.set(3, 240)
    capture.set(4, 180)
    capture.set(5, 60)
    
    origin = 0, 0
    originSet = False

    while True:
        #time1 = time.time()
        okay, image = capture.read()
        #print "Read Image: ", time.time() - time1

        if okay:
            #time2 = time.time()
            location = track(image)

            # Get origin
            if not location:
                break
            elif not originSet:
                origin = location
                originSet = True
            
            # Find delta between location and origin
            print location, origin
            delta = location[0] - origin[0], location[1] - origin[1]
            print "Delta: ", delta
            servoVal = mapValue(delta[0] + 70)
            if delta[0] == (origin[0] * -1)  - 1 and delta[1] == (origin[1] * -1) - 1:
                myServo0.write(0.057)
                print "    ERROR: ", delta, origin
            else:
                myServo0.write(servoVal)
            #print " Process: ", time.time() - time2
            if cv2.waitKey(1) & 0xFF == 27:
                break

        else:

           print('Capture failed')
           break
