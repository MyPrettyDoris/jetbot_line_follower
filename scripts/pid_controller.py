#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from jetbot_line_follower.msg import LineError
from geometry_msgs.msg import Twist
#from adafruit_motorkit import MotorKit
from Adafruit_MotorHAT import Adafruit_MotorHAT
from dynamic_reconfigure.server import Server
from jetbot_line_follower.cfg import PIDConfig
import smbus
import numpy as np

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller')

	# 初始化 Adafruit Motor HAT
        self.motor_hat = Adafruit_MotorHAT(i2c_bus=1)
        self.motor_left=self.motor_hat.getMotor(2)
        self.motor_right=self.motor_hat.getMotor(1)

        #pid param
        self.Kp = rospy.get_param('~Kp', 0.4)
        self.Ki = rospy.get_param('~Ki', 0.0)
        self.Kd = rospy.get_param('~Kd', 0.1)
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()

	# 初始化动态调参服务器
        self.srv = Server(PIDConfig, self.reconfigure_callback)

	# 订阅巡线偏差
        rospy.Subscriber('/line_error', LineError, self.error_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	# 设置关机时停止电机
        rospy.on_shutdown(self.stop_motors)

    def reconfigure_callback(self, config, level):
        # 更新 PID 参数
        self.Kp = config.Kp
        self.Ki = config.Ki
        self.Kd = config.Kd
        return config

    def error_callback(self, msg):
        # 计算时间差
        now = time.time()
        dt = now - self.last_time

        # 计算时间差
        error = msg.error

        # PID计算
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        if error < 5:
            steering = 0
        else:
            steering = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # 更新误差和时间
        self.prev_error = error
        self.last_time = now

        # 设置基础速度和转向
        base_speed = 40  # 基础速度（0-255）
        left_speed = base_speed - steering
        right_speed = base_speed + steering

        # 限幅处理（0-255）
        left_speed = max(0, min(255, left_speed))
        right_speed = max(0, min(255, right_speed))

        # 控制电机
        self.set_motors(left_speed, right_speed)

        # 发布速度指令
        twist = Twist()
        twist.linear.x = base_speed / 255.0 # 归一化到0-1
        twist.angular.z = -steering * 0.01  # 转向系数
        self.cmd_pub.publish(twist)

    def set_motors(self, left_speed, right_speed):
        # 设置左电机速度和方向
        if left_speed > 0:
            self.motor_left.run(Adafruit_MotorHAT.BACKWARD)
        else:
            self.motor_left.run(Adafruit_MotorHAT.FORWARD)
        self.motor_left.setSpeed(int(abs(left_speed)))

        # 设置右电机速度和方向
        if right_speed > 0:
            self.motor_right.run(Adafruit_MotorHAT.BACKWARD)
        else:
            self.motor_right.run(Adafruit_MotorHAT.FORWARD)
        self.motor_right.setSpeed(int(abs(right_speed)))

        # 限幅处理（Adafruit Motor HAT的输入范围为-1.0~1.0
        left_speed = np.clip(left_speed, -1.0, 1.0)
        right_speed = np.clip(right_speed, -1.0, 1.0)
	#print("left_speed")
	#print(left_speed)
	#print("right_speed")
	#print(right_speed)

    def stop_motors(self):
        # 停止所有电机
        self.motor_left.run(Adafruit_MotorHAT.RELEASE)
        self.motor_right.run(Adafruit_MotorHAT.RELEASE)

if __name__ == '__main__':
        controller = PIDController()
        rospy.spin()
        controller.stop_motors()

