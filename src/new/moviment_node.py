#!/usr/bin/env python3
import rospy
import math
import time
from std_msgs.msg import Float64, Bool
from tfg.msg import Sequencia_Torres
from tadinisdk.control.mavros.mavAPI import System
import utils

class MovimentNode():

    def __init__(self):
        rospy.init_node('Moviment_Node', anonymous=True)    

        self.drone = System()

        # Subscribers
        rospy.Subscriber('/rotation/angular_z/control_effort', Float64, self.rotation_callback)
        rospy.Subscriber('/translation/linear_y/control_effort', Float64, self.translation_callback)
        rospy.Subscriber('/tfg/sequencia_torres', Sequencia_Torres, self.target_torre_callback)

        # Publishers
        self.next_torre_msg = Bool()
        self.ditance_to_target_msg = Float64()
        self.distance_to_target_pub = rospy.Publisher('/tfg/distance_to_target', Float64, queue_size=1)
        self.next_torre_pub = rospy.Publisher('/tfg/next_torre', Bool, queue_size=1)
        
        # Timers
        rospy.Timer(rospy.Duration(0.5), self.vel_moviment)

        # Variaveis para movimentação
        self.lin_x = 0
        self.lin_y = 0
        self.lin_z = 0
        self.ang_z = 0
        self.target_latitude = 0
        self.target_longitude = 0

        # Configurações para o TimeOut
        self.start_flag = False        
        self.timeout_start = None
        self.timeout_triggered = False
    
    def timeout(self, reset):

        """
        Se parar de receber dado de velocidade por 5 manda o drone pra previous torre para cumprir o negocio certo
        """

        if self.start_flag:

            if reset:
                self.timeout_start = None
                self.timeout_triggered = False
                
            else:
                if self.timeout_start is None:
                    self.timeout_start = time.time()

                elif (time.time() - self.timeout_start) > 5 and not self.timeout_triggered: # 5 segundos é o tempo de timeout

                    print("Timeout !!")
                    self.timeout_triggered = True             
                    # Send to next GPS torre position

                    next_heading = utils.calc_next_heading(gps1=(self.previous_latitude,self.previous_longitude),
                                                           gps2=(self.target_latitude, self.target_longitude)
                    )

                    self.drone.offboard_gps_position(lat_setpoint = self.previous_latitude,
                                                    long_setpoint = self.previous_longitude,
                                                    alt_setpoint = self.previous_altitude - 800,
                                                    heading = next_heading,
                                                    precision_radius = 1)


    def velocities_calc(self):
        """
        Calcula e define as velocidade lin_x e lin_z
        """

        v_max = 2

        if self.distance_to_target_x < 4 or self.distance_to_previous_x < 4:
            self.lin_x = 0.7
        else: 
            self.lin_x = 2
        #self.lin_x = v_max * (1 / (1 + math.exp(-1 * (self.distance_to_target_x - 4))))
        self.lin_z = (self.distance_to_target_y * self.lin_x) / self.distance_to_target_x

    def rotation_callback(self, data):
        self.ang_z = data.data
        self.velocities_calc()
    
    def translation_callback(self, data):
        self.lin_y = data.data 

    def vel_moviment(self, event):
            
        if self.lin_x != 0 or self.lin_y != 0 or self.lin_z != 0 or self.ang_z != 0:

            self.timeout(reset=True)

            self.drone.offboard_velocity(linear_x = self.lin_x,
                                        linear_y = self.lin_y,
                                        linear_z = self.lin_z,
                                        angular_z = self.ang_z,
                                        ground_reference = False)
        
            self.lin_x = self.lin_y = self.lin_z = self.ang_z = 0

        else:
            self.timeout(reset=False)
    
    def target_torre_callback(self, data):
        '''
        Callback do tópico que contem as informações de prevous, target e next torre
        '''

        self.target_latitude = float(data.target.latitude.data)
        self.target_longitude = float(data.target.longitude.data)
        self.target_altitude = float(data.target.altitude.data)

        self.previous_latitude = float(data.previous.latitude.data)
        self.previous_longitude = float(data.previous.longitude.data)
        self.previous_altitude = float(data.previous.altitude.data)

        self.distance_to_previous_x = utils.gps_dist(gps1=(self.previous_latitude, self.previous_longitude),
                                                   gps2=(self.drone.get_gps.latitude, self.drone.get_gps.longitude))

        self.distance_to_target_x = utils.gps_dist(gps1=(self.target_latitude, self.target_longitude),
                                                   gps2=(self.drone.get_gps.latitude, self.drone.get_gps.longitude))
        
        self.distance_to_target_y = self.target_altitude - self.drone.get_gps.altitude

        self.ditance_to_target_msg.data = self.distance_to_target_x
        self.distance_to_target_pub.publish(self.distance_to_target_x)

        # se chegou na torre target
        if self.distance_to_target_x < 0.5:
            self.next_torre_pub.publish(True)

    def start(self):

        self.drone.arm_takeoff(20)
        self.drone.delay(20)
        self.drone.offboard_gps_position(lat_setpoint = -22.4151394,
                                    long_setpoint = -45.4483685,
                                    alt_setpoint = 12.1466 + 5,
                                    heading = 288.47,
                                    precision_radius = 1)
        self.drone.delay(10)
        self.start_flag = True
        
        rospy.spin()

if __name__ == '__main__':

    node = MovimentNode()
    node.start()

