import cv2
import numpy as np
import time
import mraa
import sys

class PID:
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.5, I=0.0, D=1.1, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator

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
    lower_orange = np.array([5,  100, 100]) # 7.40, 0.571, 0.504
    upper_orange = np.array([10, 255, 255]) # 10, 0.795, 0.415

    # Threshold the HSV image to get only orange colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5,5),0)
    
    #cv2.imwrite("pic.png", image)

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

def restrictValues(value):
    if value < 0.025:
        return 0.025
    elif value > 0.094:
        return 0.094
    else:
        return value

def findMapping(pid, start, end, lower, upper):
    val1 = pid.update(start)   # Top of the platform
    val2 = pid.update(end) # Bottom of the platform

    print "val1, val2", val1, val2

    x1 = lower # Lower bound for mapped range
    x2 = upper # Upper bound for mapped range

    #b = x2 - val2 * ((x1 - x2) / (val1 - val2))

    m = (x1 - x2) / (val1 - val2)

    b = x1 - (m * val1)
    
    print "b, m", b, m

    return b, m

def mapValue(value, b, m):
    return (value * m) + b


if __name__ == '__main__':
    myServo3 = mraa.Pwm(3)
    myServo3.period_ms(23)
    myServo3.enable(True)
    
    myServo5 = mraa.Pwm(5)
    myServo5.period_ms(23)
    myServo5.enable(True)

    p3 = PID(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
   
    p5 = PID(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

    capture = cv2.VideoCapture(0)
    capture.set(3, 240)
    capture.set(4, 180)
    #capture.set(5, 60)
    
    origin = 0, 0
    originSet = False
    goodValue = origin

    b, m = 0, 0 
    try:
        while True:
            #time1 = time.time()
            okay, image = capture.read()
            #print "Capture: ", time.time() - time1
            
            if okay:
                #time2 = time.time()
                location = track(image)
                #print "Find Ball: ", time.time() - time2

                #time3 = time.time()
                # Get origin
                if location:
                    goodValue = location
                elif location[0] == -1 and location[1] == -1:
                    location = goodValue
                if not originSet:
                    origin = location
                    originSet = True
                    p3.setPoint(origin[0])
                    p5.setPoint(origin[1])

                    b5, m5 = findMapping(p5, 5, 130, 0.066, 0.04)
                    b3, m3 = findMapping(p3, 30, 160, 0.066, 0.042)

                pid3 = p3.update(location[0])
                pid5 = p5.update(location[1])

                print "Location: ", location, " Origin: ", origin
                print "PID 3: ", pid3          
                print "PID 5: ", pid5

                servoVal3 = restrictValues(mapValue(pid3, b3, m3))
                print "Servo 3: ", servoVal3
                servoVal5 = restrictValues(mapValue(pid5, b5, m5))
                print "Servo 5: ", servoVal5
                
                #print "PID processing: ", time.time() - time3
                #if servoVal5 < 0.078:
                #    servoVal5 -= 0.02

                myServo3.write(servoVal3)
                myServo5.write(servoVal5)

            else:
                print('Capture failed')
                break
    except KeyboardInterrupt:
        print "Killing program and resetting platform"
        myServo3.write(0.052)
        myServo5.write(0.059)
        time.sleep(1)
        
