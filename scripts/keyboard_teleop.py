#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import curses

# Функция для обработки нажатий клавиш
def keypress(stdscr):
    # Инициализация ROS
    rospy.init_node('keyboard_teleop', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)  # 10hz

    twist = Twist()

    # Отключаем буферизацию ввода, делаем getch неблокирующим
    stdscr.nodelay(True)
    stdscr.clear()

    while not rospy.is_shutdown():
        try:
            key = stdscr.getch()  # Получаем код нажатой клавиши
            stdscr.refresh()

            # Выбор действия в зависимости от нажатой клавиши
            if key == curses.KEY_UP:
                twist.linear.x += 0.1
            elif key == curses.KEY_DOWN:
                twist.linear.x -= 0.1
            elif key == curses.KEY_LEFT:
                twist.angular.z += 0.1
            elif key == curses.KEY_RIGHT:
                twist.angular.z -= 0.1
            elif key == ord('s'):
                twist.linear.x = 0
                twist.angular.z = 0
            elif key == ord('q'):
                break

            pub.publish(twist)
            rate.sleep()
        except Exception as e:
            # Обработка исключений (например, при закрытии окна)
            break

# Использование curses для обработки нажатий клавиш
curses.wrapper(keypress)
