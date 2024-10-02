import csv
import rospy
#import cv2 as cv
from tadinisdk.control.mavros.mavAPI import System

rospy.init_node('tfg', anonymous=True)

csv_file = "/home/tadini/catkin_ws/src/tfg/src/coordenadas.csv"
drone = System()
drone.arm_takeoff(10)
drone.delay(10)

with open(csv_file) as read_file:
    
    csv_reader = csv.reader(read_file, delimiter=",")
    
    next(csv_reader)

    print("Inciando Missão")
    i = 1
    for row in csv_reader:

        latitude = float(row[0])
        longitude = float(row[1])
        rel_alt = float(row[2])

        drone.offboard_gps_position(latitude,longitude, rel_alt, 0, 2)
        print(f"Ponto {i} alcançado")
        drone.delay(5)
        print(f"Ponto {i} inspecionado")
        
        i+=1
    
print("Missão Finalizada, retornando a base")
drone.rtl(40)


        










