#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry  # Odometry находится в nav_msgs!
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros
import geometry_msgs.msg

class OdomTransformer:
    def __init__(self):
        rospy.init_node('odom_transformer')
        
        # VINS выдает в системе world, но оси могут быть перепутаны
        # Возможные проблемы:
        # 1. VINS использует NED (North-East-Down), а ROS использует ENU
        # 2. Камера установлена под углом
        # 3. Неправильная калибровка
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Подписчик на сырые данные VINS
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self.vins_callback)
        
        # Публикатор исправленных данных
        self.corrected_pub = rospy.Publisher('/vins_estimator/corrected_odometry', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('/vins_estimator/corrected_pose', PoseStamped, queue_size=10)
        
        # Для отладки - публикуем также в другой топик
        self.debug_pub = rospy.Publisher('/debug/transformed_odom', Odometry, queue_size=10)
        
        # Флаг для первого сообщения
        self.first_msg = True
        self.initial_position = None
        
        rospy.loginfo("Odom Transformer initialized - ожидание данных VINS...")
        
    def vins_callback(self, msg):
        # Сохраняем исходное сообщение для отладки
        original_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Фиксируем начальную позицию
        if self.first_msg:
            self.initial_position = original_pos.copy()
            self.first_msg = False
            rospy.loginfo(f"Начальная позиция VINS: {self.initial_position}")
        
        # Создаем новое сообщение Odometry
        corrected_msg = Odometry()
        corrected_msg.header = msg.header
        corrected_msg.header.frame_id = "world"  # Или "map"
        corrected_msg.child_frame_id = "vins_body"
        
        # Вариант 1: Если движение вверх отображается как движение вперед
        # Это значит, что ось Z VINS соответствует оси X ROS
        corrected_position = np.array([
            original_pos[2],   # Z VINS -> X ROS (вперед)
            -original_pos[0],  # -X VINS -> Y ROS (влево)
            -original_pos[1]   # -Y VINS -> Z ROS (вверх)
        ])
        
        # Вычитаем начальную позицию, чтобы начать с (0,0,0)
        if self.initial_position is not None:
            corrected_position[0] -= self.initial_position[2]
            corrected_position[1] -= -self.initial_position[0]
            corrected_position[2] -= -self.initial_position[1]
        
        # Применяем преобразование
        corrected_msg.pose.pose.position.x = corrected_position[0]
        corrected_msg.pose.pose.position.y = corrected_position[1]
        corrected_msg.pose.pose.position.z = corrected_position[2]
        
        # Обрабатываем ориентацию (кватернион)
        # VINS: обычно камера смотрит вперед по Z
        # ROS (MAVROS): дрон летит вперед по X
        q_orig = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # Поворот на 90 градусов вокруг оси Y, чтобы Z VINS -> X ROS
        # Это преобразует систему координат камеры в систему координат дрона
        q_rot = quaternion_from_euler(0, -np.pi/2, 0)  # -90 градусов вокруг Y
        
        q_corrected = quaternion_multiply(q_rot, q_orig)
        
        corrected_msg.pose.pose.orientation.x = q_corrected[0]
        corrected_msg.pose.pose.orientation.y = q_corrected[1]
        corrected_msg.pose.pose.orientation.z = q_corrected[2]
        corrected_msg.pose.pose.orientation.w = q_corrected[3]
        
        # Копируем ковариацию и скорость (если нужно, преобразуйте и их)
        corrected_msg.pose.covariance = msg.pose.covariance
        corrected_msg.twist = msg.twist
        
        # Преобразуем скорость аналогичным образом
        corrected_msg.twist.twist.linear.x = msg.twist.twist.linear.z
        corrected_msg.twist.twist.linear.y = -msg.twist.twist.linear.x
        corrected_msg.twist.twist.linear.z = -msg.twist.twist.linear.y
        
        # Публикуем исправленные данные
        self.corrected_pub.publish(corrected_msg)
        self.debug_pub.publish(corrected_msg)
        
        # Также публикуем как PoseStamped для удобства
        pose_msg = PoseStamped()
        pose_msg.header = corrected_msg.header
        pose_msg.pose = corrected_msg.pose.pose
        self.pose_pub.publish(pose_msg)
        
        # Логируем для отладки (только первые несколько сообщений)
        if rospy.get_time() < 10:  # Логируем первые 10 секунд
            rospy.loginfo_throttle(1.0, 
                f"VINS->ROS: Z({original_pos[2]:.2f})->X({corrected_position[0]:.2f}), "
                f"-X({-original_pos[0]:.2f})->Y({corrected_position[1]:.2f}), "
                f"-Y({-original_pos[1]:.2f})->Z({corrected_position[2]:.2f})")
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        transformer = OdomTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        pass
