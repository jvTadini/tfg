#!/usr/bin/env python3
import rospy
from tfg.msg import LineDetect
from tadinisdk.control.pid.controller import Controller


class pidNode():

    def __init__(self):

        rospy.init_node('PID_Node', anonymous=True)
        
        self.angle_controller = Controller("angle_controller", "rotation", 3, kp=0.009, ki=0.0, kd=0.0, setpoint=0.0)
        self.center_x_controller = Controller("center_x_controller", "translation", 1, kp=0.003, ki = 0.000, kd=0.0, setpoint=0)

        rospy.Subscriber('/tfg/line_info', LineDetect, self.line_detect_callback)

        rospy.spin()

    
    def line_detect_callback(self, data):

        center_x = data.center_x
        angle = data.angle
        
        self.center_x_controller.state = center_x
        self.angle_controller.state = angle

        

if __name__ == '__main__':

    node = pidNode()