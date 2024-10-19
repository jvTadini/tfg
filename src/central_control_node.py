#!/usr/bin/env python3

import rospy
import pandas as pd
from tfg.msg import Sequencia_Torres
from std_msgs.msg import Bool

class CentralControlNode():

    def __init__(self):
        rospy.init_node('Central_Node', anonymous=True)

        self.df_torres = pd.read_csv("/home/tadini/catkin_ws/src/tfg/src/torres.csv")

        self.torre_number = 1

        # Subscribers

        rospy.Subscriber('/tfg/next_torre', Bool, self.next_torre)

        # Publishers
        self.sequencia_torre_msg = Sequencia_Torres()
        self.sequencia_torre_pub = rospy.Publisher('/tfg/sequencia_torres', Sequencia_Torres, queue_size=1)


        # Timers
        rospy.Timer(rospy.Duration(0.5), self.sequencia_torre)

        rospy.spin()

    def next_torre(self, data):

        if data:
            self.torre_number += 1
        
    def sequencia_torre(self, event):

        previous_row = self.df_torres.iloc[self.torre_number - 1]
        target_row = self.df_torres.iloc[self.torre_number]
        next_row = self.df_torres.iloc[self.torre_number + 1]

        # Previous
        self.sequencia_torre_msg.previous.numero_torre.data = self.torre_number - 1
        self.sequencia_torre_msg.previous.latitude.data = previous_row['latitude']
        self.sequencia_torre_msg.previous.longitude.data = previous_row['longitude']
        self.sequencia_torre_msg.previous.altitude.data = previous_row['coords_z'] + 805 # 800 Pq foi o valor de altura inicial que eu defini nas coordenas iniciai


        # Target
        self.sequencia_torre_msg.target.numero_torre.data = self.torre_number
        self.sequencia_torre_msg.target.latitude.data = target_row['latitude']
        self.sequencia_torre_msg.target.longitude.data = target_row['longitude']
        self.sequencia_torre_msg.target.altitude.data = target_row['coords_z'] + 805 # 800 Pq foi o valor de altura inicial que eu defini nas coordenas iniciai


        # Next
        self.sequencia_torre_msg.next.numero_torre.data = self.torre_number + 1
        self.sequencia_torre_msg.next.latitude.data = next_row['latitude']
        self.sequencia_torre_msg.next.longitude.data = next_row['longitude']
        self.sequencia_torre_msg.next.altitude.data = next_row['coords_z'] + 805 # 800 Pq foi o valor de altura inicial que eu defini nas coordenas iniciai
                                                                                 # 5 pq o drone vai voar a 5 metros dos cabos

        self.sequencia_torre_pub.publish(self.sequencia_torre_msg)


if __name__ == '__main__':

    node = CentralControlNode()