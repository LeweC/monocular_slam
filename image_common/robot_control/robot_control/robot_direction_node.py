"""ToDo DocString"""
import math
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from slam_interfaces.msg import FloatArray

import numpy as np

# ToDo: copy Steffens packe into the ros package
from dynamixel_port import *

# Create an easy-to-use port to dynamixel motors
dxl = DynamixelPort()
# Before we can talk to the Dynamixel motors, we need to establish a USB connection.
# For this, we need to know the name of the USB device.
# Example names: Windows: "COM*", Linux: "/dev/ttyUSB*", MacOS: "/dev/tty.usbserial-*"
dxl.establish_connection(device_name="/dev/ttyACM0", baudrate=57600)
# Specify the motor IDs - these need to be set for each motor separately using the Dinamixel-Wizzard program before running this script.
motor_ids = [1, 2]
# First, we tell the motor to move at zero-position (value 0)
# For this, the motor has to be in POSITION_CONTROL_MODE
dxl.set_operating_mode(motor_ids, VELOCITY_CONTROL_MODE)

# Constants (adjust these based on your hardware and car specifications)
WHEEL_RADIUS = 3  # Radius of the car's wheels in centimeters
WHEELBASE_RADIUS = 10.0  # Distance between the wheels in centimeters


class DirectionSubscriber(Node):
    """ToDo DocString"""

    def __init__(self):
        super().__init__("direction_subscriber")
        self.subscription = self.create_subscription(String, "cmd_vel", self.listener_callback, 10)
        self.publisher = self.create_publisher(FloatArray, "robo_pos", 10)
        # Start motor
        dxl.set_torque_enabled(motor_ids, True)
        # Car setup
        #r = 63.8 / 2
        self.speed = 50
        self.is_turning = False
        self.goal_pos = [0, 0]
        self.world_pos = [0.0, 0.0, 0.0]
        self.motor_pos_prev = dxl.get_pos(motor_ids, multi_turn=True)
        step = 1231
        
        # tell the motor to move to zero-position
        # dxl.set_goal_pos(motor_ids, 40)
        self.pos_prev = dxl.get_pos(motor_ids, multi_turn=True)
        # wait for one second to make sure the motor has reached its goal position
        time.sleep(1)

    def listener_callback(self, msg):
        """ToDo DocString"""
        direction = msg.data
        self.drive_in_direction(direction)

    def calculate_position_orientation(self, motor_pos, motor_pos_prev, orientation_old):
        """ToDo DocString"""
        # Convert encoder values to wheel distances traveled (in centimeters)
        val_L = motor_pos[0] - self.motor_pos_prev[0]
        val_R = motor_pos[1] - self.motor_pos_prev[1]
        distance_L = (2 * math.pi * WHEEL_RADIUS * -val_L) / 4095
        distance_R = (2 * math.pi * WHEEL_RADIUS * val_R) / 4095

        orientation = (distance_L - distance_R) / 2
        orientation_rad = orientation / (WHEELBASE_RADIUS) % (2 * np.pi)
        orientation_deg = math.fmod(np.rad2deg(orientation_rad), 360)

        if abs(distance_L + distance_R) > 0.09:
            delta_x = distance_L * np.cos(np.deg2rad(orientation_old) + orientation_rad)
            delta_y = distance_L * np.sin(np.deg2rad(orientation_old) + orientation_rad)
            # print("delta_y",delta_y)
            # print("delta_x", delta_x)
        else:
            delta_x = 0
            delta_y = 0

        return delta_x, delta_y, orientation_deg

    def drive_in_direction(self, direction: String):
        """ToDo DocString"""

    
        if self.goal_pos[1] - dxl.get_pos(motor_ids, multi_turn=True)[1] < 10 and self.is_turning == True:
            ...
        else:
            self.is_turning = False
            if direction == "1":
                self.speed = 50
            elif direction == "2":
                self.speed = 100
            elif direction == "3":
                self.speed = 200
            # read arrow keys and move accordingly
            if direction == "Up":
                dxl.set_goal_vel(motor_ids, -self.speed)
            elif direction == "Down":
                dxl.set_goal_vel(motor_ids, +self.speed)
            elif direction == "Left":
                dxl.set_goal_vel(motor_ids, [self.speed, -self.speed])
            elif direction == "Right":
                dxl.set_goal_vel(motor_ids, [-self.speed, self.speed])
            elif direction == "5":
                self.is_turning = True
                self.goal_pos = dxl.get_pos(motor_ids, multi_turn=True)
                self.goal_pos[0] = self.goal_pos[0] + step
                self.goal_pos[1] = self.goal_pos[1] - step
                self.speed = 20
                dxl.set_goal_vel(motor_ids, -self.speed)
            else:
                dxl.set_goal_vel(motor_ids, 0)
            time.sleep(0.1)            
            dxl.set_goal_vel(motor_ids, 0)
        # get new position

        motor_pos = dxl.get_pos(motor_ids, multi_turn=True)

        delta_world_pos = self.calculate_position_orientation(motor_pos, self.motor_pos_prev, self.world_pos[2])

        # update x y position and orientation
        self.world_pos[0] += float(delta_world_pos[0])
        self.world_pos[1] += float(delta_world_pos[1])
        self.world_pos[2] += float(delta_world_pos[2])
        # Orientation in range of 360 dagree
        self.world_pos[2] = float(self.world_pos[2] % 360)
        # print("delta_wolrd_pos", delta_world_pos)
        print("self.world_pos: ", self.world_pos)

        self.motor_pos_prev = motor_pos
        
        float_array = FloatArray()
        float_array.elements = self.world_pos
        self.publisher.publish(float_array)
        
        """
        float_array = FloatList()
        for i in range(len(self.world_pos)):
            float_list = FloatArray()
            float_list.elements = self.world_pos[i]
            float_array.lists[i] = float_list
        self.publisher.publish(float_array)"""


def main(args=None):
    """ToDo DocString"""
    rclpy.init(args=args)

    direction_subscriber = DirectionSubscriber()

    rclpy.spin(direction_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dxl.set_torque_enabled(motor_ids, False)
    dxl.disconnect()
    direction_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
