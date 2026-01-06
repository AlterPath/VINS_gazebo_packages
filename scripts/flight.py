#!/usr/bin/env python3
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math

rospy.init_node('vins_flight_mission')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def fly_pattern():
    print("Взлет")
    navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    rospy.sleep(30)

    print("Начинаем полет по квадрату (по VINS)")
    square_points = [
        (2, 0, 0),
        (2, 2, 1.57),
        (0, 2, 3.14), 
        (0, 0, 0)    
    ]

    for x, y, yaw in square_points:
        print(f"Летим в точку x={x}, y={y}")
        navigate(x=x, y=y, z=1, yaw=yaw, frame_id='map', speed=0.1)
        rospy.sleep(30)

    print("Миссия завершена. Посадка.")
    land()

if __name__ == "__main__":
    fly_pattern()
