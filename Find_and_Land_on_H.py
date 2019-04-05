from imutils.video import VideoStream
from imutils.video import FPS
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty 
from pid import PID
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged, Ardrone3PilotingStateFlyingStateChanged
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance
from sensor_msgs.msg import Image
from imutils import contours
from skimage import measure
import cv2
import numpy as np
import imutils
import rospy
import tf
import subprocess
import os
import sys


class ThatHelipad():

    def __init__(self):
        # initialize
        
        self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10) 
        self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10) 
        self.camera_control_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
        self.fly_control_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        self.fly_control_cmd = Twist()
        self.camera_control_cmd = Twist()
        self.takeoff_cmd = Empty()                      # Takeoff
        self.land_cmd = Empty()                         # Land
        self.pitch_vel = 0                 # linear.x backward(-1) to forward(+1) 
        self.roll_vel = 0                  # linear.y right(-1) to left(+1)
        self.vertical_vel = 0              # linear.z descend(-1) to ascend(+1)
        self.rotate_vel = 0                # angular.z clockwise(-1) to anti-clockwise(+1)
        self.pid_Pitch = PID(0.04, 0.0000015, 0.0000001, -0.01, 0.01, -0.1, 0.1)
        self.pid_Roll = PID(0.03, 0.0000015, 0.0000001, -0.01, 0.01, -0.1, 0.1)
        self.pid_Vertical = PID(0.95, 0.00, 0.0, -0.01, 0.01, -0.1, 0.1)
        self.pid_Rotate = PID(0.001, 1.00, 0.001, -0.01, 0.01, -0.1, 0.1)

        self.alt_reached = False
        self.target_found = False
        self.known_distance = 145
        self.known_width = 100
        self.focalLength = 477
        self.meskicenter = (0,0)
        self.cxx = 0
        self.cyy = 0
        self.distance = 0
        self.camera_angle_x = 0
        self.camera_angle_y = 0
        self.lower_black= np.array([37,38,34]) 
        self.upper_black = np.array([117,101, 89])
        self.bridge = CvBridge()
     
        rospy.Subscriber('/bebop/image_raw', Image, self.image_callback, queue_size=1, buff_size = 2**24)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged', Ardrone3PilotingStateAltitudeChanged,
                     self.current_altitude_callback)

	rospy.spin()


    def current_altitude_callback(self, data):
	    altitude = data.altitude * 100
	    print altitude
	    if altitude < 180:
                
                print " up "
                self.fly_control_cmd.linear.x = 0
                self.fly_control_cmd.linear.y = 0
                self.fly_control_cmd.linear.z = 0.06
                self.fly_control_cmd.angular.z = 0
                self.fly_control_pub.publish(self.fly_control_cmd)		
		
	    else:
                self.alt_reached = True
                self.fly_control_cmd.linear.x = 0
                self.fly_control_cmd.linear.y = 0
                self.fly_control_cmd.linear.z = 0
                self.fly_control_cmd.angular.z = 0
		

    def image_callback(self, msg):
           
        try:           
            org_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            self.shape_recognition(org_frame)

   
    def camera_control(self):
       if self.target_found == True:     
           self.camera_control_cmd.angular.y = -90
       else:
           self.camera_control_cmd.angular.y = -60
       self.camera_control_pub.publish(self.camera_control_cmd)


    def hover(self, pitch_vel=0, roll_vel=0, vertical_vel=0, rotate_vel=0):
        self.fly_control_cmd.linear.x = pitch_vel
        self.fly_control_cmd.linear.y = roll_vel
        self.fly_control_cmd.linear.z = vertical_vel
        self.fly_control_cmd.angular.z = rotate_vel
        self.fly_control_pub.publish(self.fly_control_cmd)
       # time.sleep(s)


    def distance_to_camera(self, knownWidth, focalLength, perWidth):
	return (self.known_width * self.focalLength) / perWidth


    def shape_recognition(self, org_frame):
   
        self.camera_control()
        if self.alt_reached == True:
            
            blurred = cv2.GaussianBlur(org_frame, (7, 7), 0)
            gray = cv2.cvtColor(org_frame, cv2.COLOR_BGR2GRAY)
            binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)[1]            
            binary = cv2.erode(binary, None, iterations=2)
            binary = cv2.dilate(binary, None, iterations=4)
            labels = measure.label(binary, neighbors=8, background=0)
            mask = np.zeros(binary.shape, dtype="uint8")        
        
            for label in np.unique(labels):
	
	        if label == 0:
		    continue
         
	        labelMask = np.zeros(binary.shape, dtype="uint8")
	        labelMask[labels == label] = 255
	        numPixels = cv2.countNonZero(labelMask)
            
                if numPixels > 4000:
                                     
		    mask = cv2.add(mask, labelMask)
               
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	        cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]

            for (i, c) in enumerate(cnts):

                approx = cv2.approxPolyDP(c,0.02*cv2.arcLength(c,True),True)        
                if len(approx)==4:
                   
                    ((cX, cY), radius) = cv2.minEnclosingCircle(c)               
                    masku = cv2.inRange(blurred,self.lower_black,self.upper_black)
                    _,mnts,hie = cv2.findContours(masku,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                    for cmt in mnts:
                        rectu = cv2.boundingRect(cmt)
                        if rectu[2] < 100 or rectu[3] < 100: continue
                        ((cxx, cyy), ra) = cv2.minEnclosingCircle(cmt)                       
                        self.meskicenter = (int(cxx), int(cyy))                        
                        meski=cv2.pointPolygonTest(c, self.meskicenter, True)
                     
                        if meski > 0:
                                                  
                            marker = cv2.minAreaRect(c)
                            # focalLength = (marker[1][0] * self.known_distance) / self.known_width
                            #print("focal", focalLength)
                            self.distance = self.distance_to_camera(self.known_width, self.focalLength, marker[1][0])
                            print('distance', self.distance)
                            circle = cv2.circle(org_frame, (int(cX), int(cY)), int(radius), (0, 0, 255), 3) 
                       
                            self.roll_vel = self.pid_Roll.update(cX, 428)
	                    
                            self.fly_control_cmd.linear.x = 0.02
	                    self.fly_control_cmd.linear.y = min(max(-0.01, self.roll_vel), 0.01)
                            self.fly_control_pub.publish(self.fly_control_cmd)
                         #   print('mashaallah')
                            if self.distance < 142:
                                self.target_found = True
                                self.camera_control()
                                                                                   
                                self.roll_vel = self.pid_Roll.update(cX, 428)	                        
	                        self.pitch_vel = self.pid_Pitch.update(cY, 242)

                                self.fly_control_cmd.linear.x = min(max(-0.01, self.pitch_vel), 0.01)
	                        self.fly_control_cmd.linear.y = min(max(-0.01, self.roll_vel), 0.01)
                                self.fly_control_pub.publish(self.fly_control_cmd)
	                        #self.fly_control_cmd.linear.z = min(max(-0.01, self.vertical_vel), 0.01)
	                        #self.fly_control_cmd.angular.z = min(max(-0.01, self.rotate_vel), 0.01)
                                if (abs(cY - 240) <= 0.40) and (abs(cX - 428) <= 0.50): 
                                    print('land')
                                     self.land()
                        else:
                           #print('hover again')
                            self.hover()

                                          
        cv2.imshow('Black H On White', org_frame)        
        cv2.waitKey(1) & 0xFF 
    

def main():
    
    rospy.init_node('cooolchild', anonymous=True)   
    od = ThatHelipad()

    try:       
	pass
    except KeyboardInterrupt:
        print("Shutting down")
  

if __name__ == '__main__':

	main()

