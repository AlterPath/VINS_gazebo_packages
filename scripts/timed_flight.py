#!/usr/bin/env python3
import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class VINSStarter:
    def __init__(self):
        rospy.init_node('vins_starter')
        
        self.vins_count = 0
        self.state = State()
        self.active = False
        
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self.vins_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.rate = rospy.Rate(20)
        
    def state_callback(self, msg):
        self.state = msg
        
    def vins_callback(self, msg):
        self.vins_count += 1
        if self.vins_count > 10 and not self.active:
            self.active = True
            rospy.loginfo(f"VINS активен: {self.vins_count}")
        
    def send_pos(self, x, y, z, dur=0):
        if dur > 0:
            start = time.time()
            while time.time() - start < dur and not rospy.is_shutdown():
                self._send_single(x, y, z)
        else:
            self._send_single(x, y, z)
            
    def _send_single(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.pos_pub.publish(pose)
        self.rate.sleep()
        
    def wait_connect(self):
        while not rospy.is_shutdown() and not self.state.connected:
            rospy.sleep(0.1)
        rospy.loginfo("Подключено")
        
    def setup(self):
        for i in range(100):
            self.send_pos(0, 0, 0)
        
        try:
            self.set_mode(custom_mode="OFFBOARD")
            rospy.loginfo("OFFBOARD")
        except:
            return False
        
        rospy.sleep(1)
        
        try:
            self.arming(True)
            rospy.loginfo("Заармлен")
        except:
            return False
        
        rospy.sleep(2)
        return True
        
    def test_cycle(self):
        self.vins_count = 0
        self.active = False
        
        self.send_pos(0, 0, 1, dur=2.0)
        self.send_pos(0, 0, 0, dur=1.0)
        
        rospy.sleep(10)
        
        return self.vins_count > 5
        
    def run_mission(self):
        rospy.loginfo("Миссия начата")
        
        self.send_pos(0, 0, 1.0, dur=5.0)
        self.send_pos(2.0, 0, 1.0, dur=4.0)
        self.send_pos(2.0, 2.0, 1.0, dur=4.0)
        self.send_pos(0, 2.0, 1.0, dur=4.0)
        self.send_pos(0, 0, 1.0, dur=4.0)
        self.send_pos(0, 0, 0, dur=3.0)
        
        try:
            self.arming(False)
            rospy.loginfo("Разармлен")
        except:
            pass
            
    def main(self):
        self.wait_connect()
        
        if not self.setup():
            rospy.logerr("Ошибка настройки")
            return
            
        max_attempts = 5
        
        for attempt in range(1, max_attempts + 1):
            rospy.loginfo(f"Попытка {attempt}/{max_attempts}")
            
            if self.test_cycle():
                rospy.loginfo(f"VINS запущен за {attempt} попыток")
                self.run_mission()
                return
            else:
                rospy.loginfo(f"VINS не отвечает, сообщений: {self.vins_count}")
                
        rospy.logwarn("VINS не запущен")
        self.send_pos(0, 0, 0, dur=2.0)
        self.arming(False)

if __name__ == '__main__':
    starter = VINSStarter()
    starter.main()