#!/usr/bin/env python3
import rospy
import math
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from tadinisdk.control.pid.controller import Controller
from tadinisdk.control.mavros.mavAPI import System
from cv_bridge import CvBridge, CvBridgeError

class FollowLine:

    def __init__(self, name):
        # Initialize the ROS node for the controller
        rospy.init_node(name, anonymous=True)

        self.flag = False

        self.lat = -35.3631722 # definir parametro
        self.long = 149.1651768 # definir parametro
        self.alt = 22
        self.heading = 0
        self.precision = 2

        self.thetaList = [] 
        
        self.drone = System()

        self.angle_controller = Controller("angle_controller", "rotation", 3, kp=0.08, ki=0.0, kd=0.0, setpoint=0.0)

        self.bridge = CvBridge()    
        rospy.Subscriber('/webcam/image_raw', Image, self.camera_callback)


    def buffer(self, list, data, size):
   
      list.append(data)
      if len(list) >= size: list.pop(0)

      mean = np.mean(list)

      return(mean)

    def hough_lines(self, src):

      cdst = np.copy(src)
      dst = cv.Canny(src, 50, 200, None, 3)
      lines = cv.HoughLines(dst, 1, np.pi / 180, 200, None, 0, 0)

      theta_vector  = []
          # Draw the lines
      if lines is not None:
          for i in range(0, len(lines)):
              rho = lines[i][0][0]
              theta = lines[i][0][1]

              theta_vector.append(theta)
              
              a = math.cos(theta)
              b = math.sin(theta)
              x0 = a * rho
              y0 = b * rho
              pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
              pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
              cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)

      self.controlador(theta_vector)
      cv.imshow("Hough Lines", cdst)
      cv.imshow("Normal Image", src)

    def controlador(self, theta_vector):

      ang_z = 0
      mean = np.mean(theta_vector)
      #mean = self.buffer(self.thetaList, mean, 20)
      if mean > 1.57: mean = mean-3.14
      self.angle_controller.state = mean

      ang_z = self.angle_controller.control_effort

      if math.isnan(ang_z):
        ang_z = 0


      self.drone.offboard_velocity(1, 0, 0, ang_z, False) 
  
    def camera_callback(self, imagem):

      try:
        cv_image = self.bridge.imgmsg_to_cv2(imagem, "bgr8")
      except CvBridgeError as e:
        print(e)

      if self.flag:
        self.hough_lines(cv_image)

      if cv.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Image shutdown")
       
    def run(self):
        self.drone.arm_takeoff(20)
        self.drone.delay(20)
        self.drone.offboard_gps_position(self.lat, self.long, self.alt, self.heading, self.precision) #ver o heading
        self.drone.delay(10)
        self.flag = True

        rospy.spin()
        
    def cleanup(self):
        rospy.signal_shutdown("alou")

def main():
    
    rospy.loginfo(f"\nROS PID Node init\n")
    
    follow = FollowLine('follow_line')

    try:
        follow.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Perform cleanup and land the drone before terminating
        follow.cleanup()
    
if __name__ == '__main__':
    main()