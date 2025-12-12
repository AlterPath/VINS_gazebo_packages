#!/usr/bin/env python3
# init_vins_movement.py
import rospy
import time
import math
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

class VINSInitializer:
    def __init__(self):
        rospy.init_node('vins_initializer')
        
        # Публикатор скорости для плавного движения
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        # Сервисы
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.rate = rospy.Rate(20)
        
    def set_offboard_and_arm(self):
        """Переключение в OFFBOARD и арминг"""
        rospy.loginfo("Переключение в OFFBOARD режим...")
        
        # Сначала публикуем нулевую скорость
        for i in range(50):
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            self.vel_pub.publish(twist)
            self.rate.sleep()
        
        # Установка режима OFFBOARD
        try:
            resp = self.set_mode_client(custom_mode='OFFBOARD')
            if resp.mode_sent:
                rospy.loginfo("OFFBOARD режим установлен")
        except rospy.ServiceException as e:
            rospy.logerr(f"Ошибка установки режима: {e}")
            return False
        
        rospy.sleep(1)
        
        # Арминг
        rospy.loginfo("Арминг дрона...")
        try:
            resp = self.arming_client(True)
            if resp.success:
                rospy.loginfo("Дрон вооружен")
                return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Ошибка армирования: {e}")
        
        return False
    
    def perform_calibration_movement(self):
        """Выполнение движений для калибровки VINS"""
        rospy.loginfo("Начало калибровочных движений для VINS...")
        
        # Взлет на небольшую высоту
        rospy.loginfo("Взлет на 0.5м...")
        self.move_vertical(0.5, duration=3)
        
        # Движение вперед-назад
        rospy.loginfo("Движение вперед...")
        self.move_linear(0.2, 0, 0, duration=2)
        
        rospy.loginfo("Движение назад...")
        self.move_linear(-0.2, 0, 0, duration=2)
        
        # Движение влево-вправо
        rospy.loginfo("Движение влево...")
        self.move_linear(0, 0.2, 0, duration=2)
        
        rospy.loginfo("Движение вправо...")
        self.move_linear(0, -0.2, 0, duration=2)
        
        # Повороты для калибровки гироскопа
        rospy.loginfo("Поворот влево...")
        self.move_angular(0, 0, 0.3, duration=2)
        
        rospy.loginfo("Поворот вправо...")
        self.move_angular(0, 0, -0.3, duration=2)
        
        # Возврат в исходную точку
        rospy.loginfo("Возврат в исходную точку...")
        self.hover(2)
        
        rospy.loginfo("Калибровочные движения завершены!")
    
    def move_vertical(self, velocity_z, duration):
        """Движение по вертикали"""
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.z = velocity_z
            self.vel_pub.publish(twist)
            self.rate.sleep()
    
    def move_linear(self, vx, vy, vz, duration):
        """Линейное движение"""
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            twist.twist.linear.z = vz
            self.vel_pub.publish(twist)
            self.rate.sleep()
    
    def move_angular(self, roll_rate, pitch_rate, yaw_rate, duration):
        """Угловое движение"""
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.angular.x = roll_rate
            twist.twist.angular.y = pitch_rate
            twist.twist.angular.z = yaw_rate
            self.vel_pub.publish(twist)
            self.rate.sleep()
    
    def hover(self, duration):
        """Зависание на месте"""
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            # Нулевая скорость = зависание
            self.vel_pub.publish(twist)
            self.rate.sleep()
    
    def run(self):
        """Основной цикл"""
        rospy.loginfo("Инициализатор VINS запущен")
        
        # Ждем 5 секунд перед началом
        rospy.sleep(5)
        
        # Установка режима и арминг
        if not self.set_offboard_and_arm():
            rospy.logerr("Не удалось инициализировать дрон")
            return
        
        # Выполняем калибровочные движения
        self.perform_calibration_movement()
        
        rospy.loginfo("VINS должен быть готов к работе. Запускайте основной контроллер.")

if __name__ == '__main__':
    try:
        initializer = VINSInitializer()
        initializer.run()
    except rospy.ROSInterruptException:
        pass
