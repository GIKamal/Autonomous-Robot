#!/usr/bin/env python3

import rospy # type: ignore
from geometry_msgs.msg import Twist # type: ignore
import RPi.GPIO as GPIO # type: ignore

# Pin Definitions
#front_right
IN1 = 17
IN2 = 27
ENA = 13

#front_left
IN3 = 23
IN4 = 24
ENB = 18

#back_left
IN5 = 20
IN6 = 21
ENC = 19

#back_right
IN7 = 8
IN8 = 7
END = 12

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

GPIO.setup(IN5, GPIO.OUT)
GPIO.setup(IN6, GPIO.OUT)
GPIO.setup(ENC, GPIO.OUT)

GPIO.setup(IN7, GPIO.OUT)
GPIO.setup(IN8, GPIO.OUT)
GPIO.setup(END, GPIO.OUT)

# PWM Setup
pwm_a = GPIO.PWM(ENA, 100)
pwm_b = GPIO.PWM(ENB, 100)
pwm_c = GPIO.PWM(ENC, 100)
pwm_d = GPIO.PWM(END, 100)

pwm_a.start(0)
pwm_b.start(0)
pwm_c.start(0)
pwm_d.start(0)

def set_motor_speed(pwm, speed):
    pwm.ChangeDutyCycle(speed)

def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN5, GPIO.LOW)
    GPIO.output(IN6, GPIO.LOW)
    GPIO.output(IN7, GPIO.LOW)
    GPIO.output(IN8, GPIO.LOW)
    set_motor_speed(pwm_a, 0)
    set_motor_speed(pwm_b, 0)
    set_motor_speed(pwm_c, 0)
    set_motor_speed(pwm_d, 0)

def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN5, GPIO.HIGH)
    GPIO.output(IN6, GPIO.LOW)
    GPIO.output(IN7, GPIO.HIGH)
    GPIO.output(IN8, GPIO.LOW)
    set_motor_speed(pwm_a, 50)
    set_motor_speed(pwm_b, 50)
    set_motor_speed(pwm_c, 50)
    set_motor_speed(pwm_d, 50)

def move_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN5, GPIO.LOW)
    GPIO.output(IN6, GPIO.HIGH)
    GPIO.output(IN7, GPIO.LOW)
    GPIO.output(IN8, GPIO.HIGH)
    set_motor_speed(pwm_a, 50)
    set_motor_speed(pwm_b, 50)
    set_motor_speed(pwm_c, 50)
    set_motor_speed(pwm_d, 50)

def turn_right():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN5, GPIO.HIGH)
    GPIO.output(IN6, GPIO.LOW)
    GPIO.output(IN7, GPIO.LOW)
    GPIO.output(IN8, GPIO.HIGH)
    set_motor_speed(pwm_a, 80)
    set_motor_speed(pwm_b, 80)
    set_motor_speed(pwm_c, 80)
    set_motor_speed(pwm_d, 80)

def turn_left():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN5, GPIO.LOW)
    GPIO.output(IN6, GPIO.HIGH)
    GPIO.output(IN7, GPIO.HIGH)
    GPIO.output(IN8, GPIO.LOW)
    set_motor_speed(pwm_a, 80)
    set_motor_speed(pwm_b, 80)
    set_motor_speed(pwm_c, 80)
    set_motor_speed(pwm_d, 80)

def callback(data):
    linear_x = data.linear.x
    angular_z = data.angular.z

    if linear_x > 0:
        move_forward()
    elif linear_x < 0:
        move_backward()
    elif angular_z < 0:
        turn_right()
    elif angular_z > 0:
        turn_left()
    else:
        stop_motors()

def listener():
    rospy.init_node('motor_control', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        stop_motors()
        GPIO.cleanup()