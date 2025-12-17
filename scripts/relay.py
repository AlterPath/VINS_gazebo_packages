#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class VinsGuard:
    def __init__(self):
        rospy.init_node('vins_guard')
        self.pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/vins_estimator/odometry', Odometry, self.callback)
        self.is_broken = False

    def callback(self, msg):
        # 1. Проверка на безумные значения (NaN или слишком большие числа)
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        
        # Если координаты > 500 метров или скорость > 10 м/с - блокируем данные
        if abs(pos.x) > 500 or abs(vel.x) > 10:
            if not self.is_broken:
                rospy.logerr("VINS DIVERGED! High coordinates or velocity detected. Stopping stream.")
                self.is_broken = True
            return

        # 2. Если данные в норме, пересылаем их в MAVROS
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = "map" # PX4 ожидает 'map' или 'odom'
        pose_msg.pose = msg.pose.pose
        self.pub.publish(pose_msg)

if __name__ == '__main__':
    VinsGuard()
    rospy.spin()