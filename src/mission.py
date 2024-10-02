#!/usr/bin/env python3
import rospy
import csv
import math
import time
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from tadinisdk.control.pid.controller import Controller
from tadinisdk.control.mavros.mavAPI import System
from cv_bridge import CvBridge, CvBridgeError
import utils
from geopy.point import Point

class FollowLine:

    def __init__(self, name):
        # Initialize the ROS node for the controller
        rospy.init_node(name, anonymous=True)

        self.timeout_start = None
        self.timeout_triggered = False

        self.read_csv_input()
        self.throttle_rate = 0

        self.drone = System()

        self.angle_controller = Controller("angle_controller", "rotation", 3, kp=0.009, ki=0.0, kd=0.0, setpoint=0.0)
        self.center_x_controller = Controller("center_x_controller", "translation", 1, kp=0.00042, ki = 0.000, kd=0.0, setpoint=0)

        self.bridge = CvBridge()

        self.start = False

        rospy.Subscriber('/webcam/image_raw', Image, self.camera_callback)

    def read_csv_input(self):
       
      self.torres = []
      self.torre_number = 0

      with open('torres.csv', mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Pular o cabeçalho (primeira linha)
      
      # Ler cada linha e converter para float
        for row in reader:
            linha = [float(valor) for valor in row]
            self.torres.append(linha)
      
      # Define qual será a primeira torre a ser visitada
      self.target_torre = self.torres[self.torre_number]
      self.next_torre = self.torres[self.torre_number + 1]

    def timeout(self):
       
      # Caso o valor recebido seja 'nan' ou seja não tenha linha
      if self.start:
        if math.isnan(self.center_x) and math.isnan(self.angle):
          
          if self.timeout_start is None:
              self.timeout_start = time.time()

          elif (time.time() - self.timeout_start) > 5 and not self.timeout_triggered: # 5 segundos é o tempo de timeout

            print("Timeout !!")
            self.timeout_triggered = True             
            # Send to next GPS torre position

            next_heading = utils.calc_next_heading(self.target_torre[3:5],
                                                  self.next_torre[3:5])

            self.drone.offboard_gps_position(lat_setpoint = self.target_torre[3],
                                              long_setpoint = self.target_torre[4],
                                              alt_setpoint = self.target_torre[2] + 5,
                                              heading = next_heading,
                                              precision_radius = 1)
            
        else: 
          self.timeout_start = None
          self.timeout_triggered = False

    def proxima_torre(self):

      # Calcular a diferença d altura entre as duas torres para definir o throttle rate
      # Nesse caso target é a torre que eu acabei de chegar que era a target antes
      self.throttle_rate = (self.next_torre[2] - self.target_torre[2]) / (self.next_torre[5] / 2)
      self.torre_number += 1
      self.target_torre = self.next_torre
      self.next_torre = self.torres[self.torre_number + 1]
       
    def controler(self):

      lin_x = lin_y = ang_z = lin_z = 0

      if not math.isnan(self.center_x) and not math.isnan(self.angle):
        self.center_x_controller.state = self.center_x 
        self.angle_controller.state = self.angle
        
        if abs(self.angle) > 15:
          lin_x = 0
          lin_z = 0
        else:
          lin_x = 2
          lin_z = self.throttle_rate

        lin_y = self.center_x_controller.control_effort
        ang_z = self.angle_controller.control_effort

        self.drone.offboard_velocity(linear_x = lin_x,
                                    linear_y = lin_y,
                                    linear_z = lin_z,
                                    angular_z = ang_z,
                                    ground_reference = False) 
        
      current_gps = (self.drone.get_gps.latitude, self.drone.get_gps.longitude)

      distancia_torre = utils.gps_dist(gps1 = current_gps,
                                        gps2 = self.target_torre[3:5])
      
      if distancia_torre <= 1:
         self.proxima_torre()
       
    def img_process(self):

      self.center_x, self.angle = utils.hough_lines(self.cv_image)
      self.timeout()
      self.controler()

    def camera_callback(self, imagem):

      try:
        self.cv_image = self.bridge.imgmsg_to_cv2(imagem, "bgr8")
      except CvBridgeError as e:
        print(e)

      if self.cv_image is not None:
         self.img_process()

      if cv.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Image shutdown")
       
    def run(self):
        
        self.drone.arm_takeoff(20)
        self.drone.delay(20)
        self.start = True
        rospy.spin()
        
    def cleanup(self):
        rospy.signal_shutdown("alou")

def main():
    
    rospy.loginfo(f"\nROS PID Node init\n")
    
    follow = FollowLine('follow_line')

    try:
        follow.run()
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        # Perform cleanup and land the drone before terminating
        follow.cleanup()
    
if __name__ == '__main__':
    main()