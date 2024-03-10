#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

def move_forward():
    # Инициализация узла ROS
    rospy.init_node('robot_mover', anonymous=True)
    # Публикатор команд скорости
    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Проверка, не был ли узел ROS остановлен
    while not rospy.is_shutdown():
        # Создание сообщения типа Twist
        vel_msg = Twist()
        
        # Задание линейной скорости вперёд (x>0)
        vel_msg.linear.x = 1.0 # Можно изменить скорость в соответствии с требованиями
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        
        # Угловая скорость равна 0 для движения прямо
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.5
        
        # Отправка сообщения
        vel_publisher.publish(vel_msg)
        
        # Ждём немного, чтобы увидеть эффект
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
