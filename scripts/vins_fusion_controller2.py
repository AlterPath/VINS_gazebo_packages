import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest
# CommandBoolRequest не требуется, передаем аргумент напрямую

# --- Параметры Полета ---
TAKEOFF_ALT = 1.5           # Целевая высота взлета (метры)
FORWARD_DIST = 5.0          # Расстояние вперед (метры)
LEFT_DIST = 1.0             # Расстояние влево (метры)
POS_TOLERANCE = 0.3         # Допуск достижения цели (метры)
RATE = 20.0                 # Частота основного цикла (Гц)

class VINSController:
    """
    Управляющая нода для выполнения полетной миссии, основанной на 
    данных одометрии от VINS-Fusion и командах MAVROS.
    """
    def __init__(self):
        rospy.init_node('vins_fusion_controller', anonymous=True)

        # --- Состояния ---
        self.current_vins_pose = Point() # Текущая позиция VINS-Fusion (x, y, z)
        self.mavros_state = State()      # Текущее состояние MAVROS (armed, mode)
        self.target_position = PoseStamped() # Текущая целевая позиция для MAVROS

        # --- Топики (Вход) ---
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self.vins_odom_cb, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.mavros_state_cb, queue_size=1)

        # --- Топики (Выход) ---
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # --- Сервисы MAVROS ---
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(RATE)
        
        # Инициализация целевой позиции
        self.target_position.header.frame_id = "local_origin"
        self.target_position.pose.position.x = 0
        self.target_position.pose.position.y = 0
        self.target_position.pose.position.z = 0
        
        self.main_loop()

    # --- Обработчики Топиков ---
    def vins_odom_cb(self, msg):
        """Обработчик одометрии VINS-Fusion."""
        # Используем только позицию (x, y, z) из сообщения Odometry
        self.current_vins_pose = msg.pose.pose.position

    def mavros_state_cb(self, msg):
        """Обновляет текущее состояние MAVROS."""
        self.mavros_state = msg

    # --- Вспомогательные Функции ---
    def set_target(self, x, y, z):
        """Устанавливает новую целевую позицию и готовит ее к публикации."""
        self.target_position.pose.position.x = x
        self.target_position.pose.position.y = y
        self.target_position.pose.position.z = z
        rospy.loginfo(f"Установлена цель: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

    def is_target_reached(self):
        """Проверяет, достиг ли дрон целевой позиции, используя данные VINS."""
        dx = self.target_position.pose.position.x - self.current_vins_pose.x
        dy = self.target_position.pose.position.y - self.current_vins_pose.y
        dz = self.target_position.pose.position.z - self.current_vins_pose.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance < POS_TOLERANCE

    def wait_for_reach(self, timeout=30.0):
        """Ожидает достижения цели с постоянной отправкой setpoint."""
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            # Постоянная публикация целевой позиции
            self.target_position.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(self.target_position)

            if self.is_target_reached():
                rospy.loginfo("Цель достигнута!")
                return True
            
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("Таймаут ожидания достижения цели!")
                return False

            self.rate.sleep()

    # --- Основная Логика Полета ---
    def main_loop(self):
        
        rospy.loginfo("Ожидание MAVROS и данных VINS-Fusion...")
        # Убедимся, что получаем данные одометрии
        while not rospy.is_shutdown() and abs(self.current_vins_pose.x) < 0.001:
            self.rate.sleep()
        rospy.loginfo("Получены данные VINS. Инициализация полета.")

        # Шаг 0: Подготовка к OFFBOARD и Армирование
        offb_set_mode = SetModeRequest(custom_mode='OFFBOARD')
        last_req = rospy.Time.now()
        
        # Предварительная отправка setpoint для MAVROS
        for i in range(100):
            self.target_position.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(self.target_position)
            self.rate.sleep()

        # Цикл установки OFFBOARD и армирования
        while not rospy.is_shutdown():
            # Попытка переключиться в OFFBOARD
            if self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD включен")
                last_req = rospy.Time.now()
            
            # Попытка армирования (ИСПРАВЛЕНИЕ: value=True передается как аргумент)
            if not self.mavros_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.arming_client(value=True).success: 
                    rospy.loginfo("Дрон армирован")
                last_req = rospy.Time.now()
            
            if self.mavros_state.mode == "OFFBOARD" and self.mavros_state.armed:
                break
            
            # Продолжаем публиковать setpoint, пока не перейдем в OFFBOARD
            self.target_position.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(self.target_position)
            self.rate.sleep()
        
        rospy.loginfo("Армирование и режим OFFBOARD успешно установлены. НАЧАЛО МИССИИ.")

        # --- МИССИЯ ---
        
        # 1. Взлет
        rospy.loginfo("1. Взлет на высоту %.2f м...", TAKEOFF_ALT)
        # Сохраняем текущие X, Y (считанные VINS), летим на целевую высоту
        self.set_target(self.current_vins_pose.x, self.current_vins_pose.y, TAKEOFF_ALT)
        self.wait_for_reach(timeout=10.0)
        
        # 2. 5 метров вперед (относительно начальной позиции VINS)
        initial_x = self.current_vins_pose.x
        initial_y = self.current_vins_pose.y
        rospy.loginfo(f"2. Летим вперед на {FORWARD_DIST} м (в кадре VINS).")
        
        target_x1 = initial_x + FORWARD_DIST # Перемещение по X (вперед)
        target_y1 = initial_y 
        self.set_target(target_x1, target_y1, TAKEOFF_ALT)
        self.wait_for_reach(timeout=20.0)

        # 3. 1 метр влево (относительно VINS-кадра)
        rospy.loginfo(f"3. Летим влево на {LEFT_DIST} м (в кадре VINS).")
        target_x2 = target_x1
        target_y2 = target_y1 + LEFT_DIST # Перемещение по Y (влево)
        self.set_target(target_x2, target_y2, TAKEOFF_ALT)
        self.wait_for_reach(timeout=20.0)
        
        # 4. Посадка
        rospy.loginfo("4. Завершение миссии. Переключение в режим LAND (AUTO.LAND).")
        land_set_mode = SetModeRequest(custom_mode='AUTO.LAND')
        self.set_mode_client(land_set_mode)
        
        rospy.loginfo("Ожидание завершения посадки (Disarmed state)...")
        
        # ОЖИДАЕМ СИГНАЛА MAVROS, что дрон обесточен (Disarmed), игнорируя дрейф VINS
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if not self.mavros_state.armed:
                rospy.loginfo("Дрон успешно приземлился и обесточен (Disarmed).")
                break
                
            if (rospy.Time.now() - start_time) > rospy.Duration(30.0):
                rospy.logwarn("Таймаут ожидания посадки.")
                break

            self.rate.sleep()
            
        rospy.loginfo("Миссия завершена. Завершение ноды.")

if __name__ == '__main__':
    try:
        VINSController()
    except rospy.ROSInterruptException:
        pass
