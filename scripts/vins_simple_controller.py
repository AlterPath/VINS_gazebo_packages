#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from clover import srv

class SimpleVINSController:
    def __init__(self):
        rospy.init_node('simple_vins_controller')
        
        # Wait for services
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/cmd/takeoff')
        rospy.wait_for_service('get_telemetry')
        rospy.wait_for_service('navigate')
        
        self.arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.takeoff = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        
        # VINS odometry
        self.vins_position = None
        rospy.Subscriber('/vins_estimator/odometry', TwistStamped, self.vins_callback)
        
        # Mission parameters
        self.target_height = 1.5
        self.forward_distance = 5.0
        self.left_distance = 1.0
        
    def vins_callback(self, msg):
        """Get VINS odometry"""
        self.vins_position = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        
    def wait_for_vins(self, timeout=10.0):
        """Wait for VINS to initialize"""
        start_time = rospy.Time.now()
        while self.vins_position is None and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("VINS initialization timeout")
                return False
            rospy.sleep(0.1)
        rospy.loginfo("VINS initialized")
        return True
    
    def set_offboard_mode(self):
        """Set OFFBOARD mode"""
        try:
            resp = self.set_mode(custom_mode="OFFBOARD")
            rospy.loginfo(f"Set OFFBOARD mode: {resp.mode_sent}")
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"Set mode failed: {e}")
            return False
    
    def arm(self):
        """Arm the drone"""
        try:
            resp = self.arming(True)
            rospy.loginfo(f"Arming: {resp.success}")
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Arming failed: {e}")
            return False
    
    def takeoff_to_height(self, height):
        """Takeoff to specified height"""
        rospy.loginfo(f"Taking off to {height}m")
        
        # Set mode to OFFBOARD
        if not self.set_offboard_mode():
            return False
        
        # Arm the drone
        if not self.arm():
            return False
        
        # Takeoff using navigate service
        try:
            self.navigate(x=0, y=0, z=height, frame_id='body', auto_arm=True)
            rospy.sleep(5)
            
            # Wait for takeoff completion
            for i in range(20):
                telem = self.get_telemetry(frame_id='body')
                if abs(telem.z - height) < 0.1:
                    rospy.loginfo("Takeoff complete")
                    return True
                rospy.sleep(0.5)
            
            rospy.logwarn("Takeoff timeout")
            return False
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Takeoff failed: {e}")
            return False
    
    def navigate_relative(self, dx, dy, dz, speed=0.5):
        """Navigate relative to current VINS position"""
        if self.vins_position is None:
            rospy.logerr("VINS not initialized")
            return False
        
        target = self.vins_position + np.array([dx, dy, dz])
        rospy.loginfo(f"Navigating to: {target}")
        
        try:
            self.navigate(x=target[0], y=target[1], z=target[2], 
                         speed=speed, frame_id='body')
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Navigation failed: {e}")
            return False
    
    def wait_for_position(self, timeout=10.0, tolerance=0.2):
        """Wait to reach target position"""
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Position timeout")
                return False
            rospy.sleep(0.1)
        return True
    
    def run_mission(self):
        """Execute the mission: 5m forward, 1m left"""
        rospy.loginfo("Starting VINS mission")
        
        # 1. Takeoff
        if not self.takeoff_to_height(self.target_height):
            rospy.logerr("Takeoff failed")
            return
        
        rospy.sleep(3)  # Stabilize
        
        # 2. Wait for VINS
        if not self.wait_for_vins():
            rospy.logerr("VINS not ready")
            return
        
        # 3. Fly 5m forward using VINS odometry
        rospy.loginfo("Flying 5m forward")
        self.navigate_relative(self.forward_distance, 0, 0)
        self.wait_for_position()
        rospy.sleep(2)
        
        # 4. Fly 1m left
        rospy.loginfo("Flying 1m left")
        self.navigate_relative(0, self.left_distance, 0)
        self.wait_for_position()
        rospy.sleep(2)
        
        # 5. Return to start (relative to initial VINS position)
        rospy.loginfo("Returning to start")
        self.navigate_relative(-self.forward_distance, -self.left_distance, 0)
        self.wait_for_position()
        rospy.sleep(2)
        
        # 6. Land
        rospy.loginfo("Landing")
        self.navigate(x=0, y=0, z=0.1, frame_id='body')
        rospy.sleep(5)
        
        rospy.loginfo("Mission complete!")

if __name__ == '__main__':
    try:
        controller = SimpleVINSController()
        rospy.sleep(2)  # Wait for initialization
        controller.run_mission()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted")
