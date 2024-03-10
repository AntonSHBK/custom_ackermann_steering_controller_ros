#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class AckermannSteeringController:
    def __init__(self):
        self.name = '/ackermann_steering_controller'
        
        rospy.init_node('ackermann_steering_controller', anonymous=True)

        # Загрузка параметров из конфигурационного файла
        self.wheel_base = rospy.get_param(self.name+'/wheel_base', 1.0)
        self.cmd_vel_topic = rospy.get_param(self.name+'/cmd_vel_topic', '/cmd_vel')
        # self.steering_angle_topic = rospy.get_param(self.name+'/steering_angle_topic', '/steering_angle')
        self.wheel_track = rospy.get_param(self.name+'/wheel_track', 0.5)
        self.max_steering_angle = rospy.get_param(self.name+'/max_steering_angle', 0.7854)

        # Подписка на топик с командами управления
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)
        
        # Publishers для отправки команд на контроллеры колес
        self.front_left_wheel_pub = rospy.Publisher(self.name+'/front_left_wheel_position_controller/command', Float64, queue_size=1)
        self.front_right_wheel_pub = rospy.Publisher(self.name+'/front_right_wheel_position_controller/command', Float64, queue_size=1)
        self.rear_left_wheel_pub = rospy.Publisher(self.name+'/rear_left_wheel_velocity_controller/command', Float64, queue_size=1)
        self.rear_right_wheel_pub = rospy.Publisher(self.name+'/rear_right_wheel_velocity_controller/command', Float64, queue_size=1)

    def cmd_vel_callback(self, data):
        # Получение линейной скорости и угловой скорости
        linear_velocity = data.linear.x
        angular_velocity = data.angular.z

        
        # Вычисление радиуса поворота из угловой и линейной скорости
        if angular_velocity == 0 or linear_velocity == 0:
            radius = float('inf')
        else:
            radius = linear_velocity / angular_velocity            
            
        # Вычисление углов поворота для передних колес
        left_steering_angle, right_steering_angle = self.calculate_steering_angles(radius)

        # Адаптация скорости задних колес
        left_rear_wheel_velocity, right_rear_wheel_velocity = self.calculate_wheel_velocities(radius, linear_velocity)

        # Публикация углов поворота и скоростей
        self.front_left_wheel_pub.publish(Float64(left_steering_angle))
        self.front_right_wheel_pub.publish(Float64(right_steering_angle))
        self.rear_left_wheel_pub.publish(Float64(left_rear_wheel_velocity))
        self.rear_right_wheel_pub.publish(Float64(right_rear_wheel_velocity))
        

    def calculate_steering_angles(self, radius):
        if radius == float('inf'):
            return 0.0, 0.0
        left_angle = math.atan(self.wheel_base / (radius + self.wheel_track / 2))
        right_angle = math.atan(self.wheel_base / (radius - self.wheel_track / 2))
        return left_angle, right_angle

    def calculate_wheel_velocities(self, radius, linear_velocity):
        if radius == float('inf'):
            return linear_velocity, linear_velocity
        left_velocity = linear_velocity * (radius - self.wheel_track / 2) / radius
        right_velocity = linear_velocity * (radius + self.wheel_track / 2) / radius
        return left_velocity, right_velocity


if __name__ == '__main__':
    try:
        controller = AckermannSteeringController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
