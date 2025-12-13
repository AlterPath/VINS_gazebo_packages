#!/usr/bin/env python3
import rospy
import math
import time
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import CommandBool, SetMode

class PID:
    """Класс PID-регулятора для стабилизации углов"""
    def __init__(self, kp, ki, kd, max_output=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = -max_output
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        
    def compute(self, error):
        current_time = time.time()
        
        if self.previous_time is None:
            self.previous_time = current_time
            dt = 0.02  # 50 Гц
        else:
            dt = current_time - self.previous_time
            if dt <= 0:
                dt = 0.02
        
        # Пропорциональная составляющая
        p = self.kp * error
        
        # Интегральная составляющая
        self.integral += error * dt
        i = self.ki * self.integral
        
        # Дифференциальная составляющая
        d = self.kd * (error - self.previous_error) / dt if dt > 0 else 0
        
        # Суммарный выход
        output = p + i + d
        
        # Ограничение выхода (anti-windup)
        if output > self.max_output:
            output = self.max_output
            self.integral -= error * dt  # anti-windup
        elif output < self.min_output:
            output = self.min_output
            self.integral -= error * dt  # anti-windup
            
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def reset(self):
        """Сброс интегральной составляющей"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None

class StabilizedDroneIMU:
    def __init__(self):
        rospy.init_node('stabilized_drone_imu_pid')
        
        # Сервисы Clover
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.land = rospy.ServiceProxy('land', Trigger)
        
        # MAVROS сервисы
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # Издатель для управления ориентацией
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Подписка на IMU
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback)
        
        # Подписка на глобальную позицию
        self.position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        
        # Текущие углы Эйлера
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # Текущая позиция
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        
        # Начальная позиция
        self.start_x = None
        self.start_y = None
        self.start_z = None
        
        # PID регуляторы для стабилизации
        self.pid_roll = PID(kp=1.5, ki=0.02, kd=0.25, max_output=0.2)  # Уменьшены коэффициенты
        self.pid_pitch = PID(kp=1.5, ki=0.02, kd=0.25, max_output=0.2)
        
        # PID регуляторы для позиционирования (новые)
        self.pid_x = PID(kp=0.8, ki=0.01, kd=0.15, max_output=0.3)
        self.pid_y = PID(kp=0.8, ki=0.01, kd=0.15, max_output=0.3)
        self.pid_z = PID(kp=1.2, ki=0.05, kd=0.2, max_output=0.5)
        
        # Флаги
        self.imu_ready = False
        self.position_ready = False
        self.initial_yaw = None
        
        # Тайминги
        self.takeoff_duration = 5.0
        self.hover_duration = 10.0
        self.land_duration = 5.0
        
        # Таймер для вывода координат
        self.last_print_time = time.time()
        self.print_interval = 1.0  # Выводить каждую секунду
        
    def imu_callback(self, msg):
        """Обработчик данных IMU"""
        try:
            orientation_q = msg.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)
            
            if self.initial_yaw is None:
                self.initial_yaw = self.yaw
                
            self.imu_ready = True
        except Exception as e:
            rospy.logerr(f"Error processing IMU data: {e}")
    
    def position_callback(self, msg):
        """Обработчик позиции дрона"""
        try:
            self.current_x = msg.pose.position.x
            self.current_y = msg.pose.position.y
            self.current_z = msg.pose.position.z
            
            # Сохраняем стартовую позицию при первом получении
            if self.start_x is None:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_z = self.current_z
                rospy.loginfo(f"Start position set: X={self.start_x:.2f}, Y={self.start_y:.2f}, Z={self.start_z:.2f}")
            
            self.position_ready = True
            
            # Вывод координат каждую секунду
            current_time = time.time()
            if current_time - self.last_print_time >= self.print_interval:
                rospy.loginfo(f"POSITION: X={self.current_x:.2f}m, Y={self.current_y:.2f}m, Z={self.current_z:.2f}m, "
                            f"Roll={math.degrees(self.roll):.1f}°, Pitch={math.degrees(self.pitch):.1f}°, "
                            f"Yaw={math.degrees(self.yaw):.1f}°")
                self.last_print_time = current_time
                
        except Exception as e:
            rospy.logerr(f"Error processing position data: {e}")
    
    def wait_for_sensors(self, timeout=10):
        """Ожидание данных от всех сенсоров"""
        start_time = time.time()
        
        while time.time() - start_time < timeout and not rospy.is_shutdown():
            if self.imu_ready and self.position_ready:
                rospy.loginfo("All sensors ready")
                return True
            
            if not self.imu_ready:
                rospy.loginfo_throttle(1, "Waiting for IMU data...")
            if not self.position_ready:
                rospy.loginfo_throttle(1, "Waiting for position data...")
                
            rospy.sleep(0.1)
        
        rospy.logwarn("Sensors not fully available")
        return False
    
    def set_drone_mode(self, mode="OFFBOARD"):
        """Установка режима полета"""
        try:
            rospy.loginfo(f"Setting mode to {mode}")
            response = self.set_mode(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Mode {mode} set successfully")
            return response.mode_sent
        except Exception as e:
            rospy.logerr(f"Error setting mode: {e}")
            return False
    
    def arm_drone(self):
        """Взведение моторов"""
        try:
            rospy.loginfo("Arming motors...")
            response = self.arming(True)
            if response.success:
                rospy.loginfo("Motors armed successfully")
            return response.success
        except Exception as e:
            rospy.logerr(f"Error arming motors: {e}")
            return False
    
    def takeoff_sequence(self):
        """Последовательность взлета с использованием PID для позиционирования"""
        rospy.loginfo("=== TAKEOFF SEQUENCE ===")
        
        # 1. Переключение в OFFBOARD
        self.set_drone_mode("OFFBOARD")
        rospy.sleep(1)
        
        # 2. Взведение моторов
        self.arm_drone()
        rospy.sleep(1)
        
        # 3. Плавный взлет с контролем позиции
        start_time = time.time()
        rate = rospy.Rate(30)
        
        # Целевая высота
        target_z = 1.5
        
        while time.time() - start_time < self.takeoff_duration and not rospy.is_shutdown():
            # Прогресс взлета
            progress = (time.time() - start_time) / self.takeoff_duration
            
            # Плавное увеличение целевой высоты
            current_target_z = target_z * progress
            
            # Ошибки позиционирования
            error_x = self.current_x - self.start_x
            error_y = self.current_y - self.start_y
            error_z = current_target_z - self.current_z
            
            # Коррекция от PID контроллеров
            x_correction = -self.pid_x.compute(error_x)
            y_correction = -self.pid_y.compute(error_y)
            z_correction = self.pid_z.compute(error_z)
            
            # Стабилизация углов (держим дрон ровно)
            roll_correction = self.pid_roll.compute(-self.roll)
            pitch_correction = self.pid_pitch.compute(-self.pitch)
            
            # Комбинируем коррекции
            # Для взлета в основном используем вертикальное управление
            final_x = x_correction * 0.1  # Меньший вклад горизонтальной коррекции
            final_y = y_correction * 0.1
            final_z = z_correction
            
            # Отправляем команду в системе координат тела дрона
            try:
                self.navigate(
                    x=final_x, 
                    y=final_y, 
                    z=final_z, 
                    yaw=float('nan'),  # Сохраняем текущее рыскание
                    speed=0.3,  # Уменьшена скорость для плавности
                    frame_id='body',  # Координаты относительно тела дрона
                    auto_arm=False
                )
            except Exception as e:
                rospy.logerr(f"Navigate error: {e}")
            
            # Логирование
            rospy.loginfo_throttle(0.5,
                f"Takeoff: {progress*100:.0f}% | "
                f"Target Z: {current_target_z:.2f}m | "
                f"Errors: X={error_x:.3f}, Y={error_y:.3f}, Z={error_z:.3f} | "
                f"Angles: R={math.degrees(self.roll):.1f}° P={math.degrees(self.pitch):.1f}°"
            )
            
            rate.sleep()
        
        rospy.loginfo("Takeoff complete")
        
        # Сброс интегральных составляющих после взлета
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
    
    def hover_sequence(self):
        """Последовательность висения с удержанием позиции"""
        rospy.loginfo("=== HOVER SEQUENCE ===")
        
        start_time = time.time()
        rate = rospy.Rate(30)
        
        # Целевая позиция (текущая позиция после взлета)
        target_x = self.current_x
        target_y = self.current_y
        target_z = 1.5  # Целевая высота
        
        while time.time() - start_time < self.hover_duration and not rospy.is_shutdown():
            # Ошибки позиционирования
            error_x = target_x - self.current_x
            error_y = target_y - self.current_y
            error_z = target_z - self.current_z
            
            # Коррекция от PID контроллеров
            x_correction = self.pid_x.compute(error_x)
            y_correction = self.pid_y.compute(error_y)
            z_correction = self.pid_z.compute(error_z)
            
            # Стабилизация углов
            roll_correction = self.pid_roll.compute(-self.roll)
            pitch_correction = self.pid_pitch.compute(-self.pitch)
            
            # Комбинируем коррекции
            # Добавляем небольшую коррекцию от углов для компенсации дрейфа
            combined_x = x_correction - pitch_correction * 0.05
            combined_y = y_correction + roll_correction * 0.05
            combined_z = z_correction
            
            # Отправляем команду
            try:
                self.navigate(
                    x=combined_x,
                    y=combined_y,
                    z=combined_z,
                    yaw=float('nan'),
                    speed=0.15,  # Очень медленная скорость для точного висения
                    frame_id='body',
                    auto_arm=False
                )
            except Exception as e:
                rospy.logerr(f"Navigate error: {e}")
            
            # Логирование
            elapsed = time.time() - start_time
            rospy.loginfo_throttle(1,
                f"Hover: {elapsed:.1f}/{self.hover_duration}s | "
                f"Target: X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f} | "
                f"Current: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}"
            )
            
            rate.sleep()
        
        rospy.loginfo("Hover complete")
        
        # Сброс интегральных составляющих перед посадкой
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
    
    def land_sequence(self):
        """Последовательность посадки с контролем позиции"""
        rospy.loginfo("=== LANDING SEQUENCE ===")
        
        start_time = time.time()
        rate = rospy.Rate(30)
        
        # Начальная высота
        initial_z = self.current_z
        target_x = self.start_x  # Пытаемся вернуться в стартовую точку
        target_y = self.start_y
        
        while time.time() - start_time < self.land_duration and not rospy.is_shutdown():
            # Прогресс посадки
            progress = (time.time() - start_time) / self.land_duration
            
            # Плавное уменьшение целевой высоты
            current_target_z = initial_z * (1 - progress)
            
            # Не позволяем цели уйти ниже 0.1 метра
            if current_target_z < 0.1:
                current_target_z = 0.1
            
            # Ошибки позиционирования
            error_x = target_x - self.current_x
            error_y = target_y - self.current_y
            error_z = current_target_z - self.current_z
            
            # Коррекция от PID контроллеров
            x_correction = self.pid_x.compute(error_x)
            y_correction = self.pid_y.compute(error_y)
            z_correction = self.pid_z.compute(error_z)
            
            # Стабилизация углов
            roll_correction = self.pid_roll.compute(-self.roll)
            pitch_correction = self.pid_pitch.compute(-self.pitch)
            
            # Комбинируем коррекции
            combined_x = x_correction * 0.8  # Уменьшаем влияние горизонтальной коррекции при посадке
            combined_y = y_correction * 0.8
            combined_z = z_correction
            
            # Отправляем команду
            try:
                self.navigate(
                    x=combined_x,
                    y=combined_y,
                    z=combined_z,
                    yaw=float('nan'),
                    speed=0.2,
                    frame_id='body',
                    auto_arm=False
                )
            except Exception as e:
                rospy.logerr(f"Navigate error: {e}")
            
            # Логирование
            rospy.loginfo_throttle(0.5,
                f"Landing: {progress*100:.0f}% | "
                f"Target Z: {current_target_z:.2f}m | "
                f"Current Z: {self.current_z:.2f}m | "
                f"Errors: X={error_x:.3f}, Y={error_y:.3f}"
            )
            
            rate.sleep()
        
        rospy.loginfo("Landing descent complete")
    
    def run_mission(self):
        """Основная программа полета"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("STABILIZED DRONE MISSION - IMPROVED PID CONTROL")
        rospy.loginfo("=" * 50)
        
        try:
            # Шаг 0: Ожидание всех сенсоров
            rospy.loginfo("Step 0: Initializing sensors...")
            if not self.wait_for_sensors():
                rospy.logerr("Sensors initialization failed")
                return
            
            rospy.sleep(2)
            
            # Вывод начальных координат
            rospy.loginfo(f"Initial position: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}")
            rospy.loginfo(f"Initial angles: Roll={math.degrees(self.roll):.1f}°, "
                         f"Pitch={math.degrees(self.pitch):.1f}°, "
                         f"Yaw={math.degrees(self.yaw):.1f}°")
            
            # Шаг 1: Взлет (5 секунд)
            self.takeoff_sequence()
            
            # Короткая пауза
            rospy.sleep(0.5)
            
            # Шаг 2: Висение (10 секунд)
            self.hover_sequence()
            
            # Короткая пауза
            rospy.sleep(0.5)
            
            # Шаг 3: Посадка (5 секунд)
            self.land_sequence()
            
            # Шаг 4: Финальная посадка
            rospy.loginfo("Step 4: Final landing...")
            try:
                self.land()
                rospy.loginfo("Land command sent")
            except Exception as e:
                rospy.logerr(f"Land service error: {e}")
            
            # Вывод финальных координат
            rospy.loginfo(f"Final position: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}")
            
            # Небольшая задержка перед отключением
            rospy.sleep(2)
            
            rospy.loginfo("=" * 50)
            rospy.loginfo("MISSION COMPLETED SUCCESSFULLY!")
            rospy.loginfo("=" * 50)
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Mission interrupted")
        except Exception as e:
            rospy.logerr(f"Mission failed: {e}")
            # Экстренная посадка
            try:
                self.land()
            except:
                pass
    
    def simple_takeoff_test(self):
        """Простой тест взлета и посадки для отладки"""
        rospy.loginfo("SIMPLE TAKEOFF TEST")
        
        if not self.wait_for_sensors():
            return
        
        rospy.loginfo(f"Starting at: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}")
        
        # Взлет на 1 метр
        rospy.loginfo("Taking off to 1 meter...")
        try:
            self.navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
            rospy.sleep(2)
        except Exception as e:
            rospy.logerr(f"Takeoff error: {e}")
            return
        
        # Вывод текущей позиции
        rospy.loginfo(f"After takeoff: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}")
        
        # Висение 3 секунды
        rospy.loginfo("Hovering for 3 seconds...")
        rospy.sleep(3)
        
        # Посадка
        rospy.loginfo("Landing...")
        try:
            self.land()
        except Exception as e:
            rospy.logerr(f"Land error: {e}")
        
        rospy.loginfo("Test complete")

if __name__ == '__main__':
    # Даем ROS время на инициализацию
    rospy.sleep(3.0)
    
    # Создаем контроллер
    controller = StabilizedDroneIMU()
    
    # Тестовые варианты запуска:
    
    # Вариант 1: Простой тест (рекомендуется сначала)
    controller.simple_takeoff_test()
    
    # Вариант 2: Полная миссия с PID
    # controller.run_mission()
    
    rospy.spin()