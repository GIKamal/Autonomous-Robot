#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# Pin Definitions
# Front Right
IN1 = 17  # Direction pin 1
IN2 = 27  # Direction pin 2
ENA = 13  # PWM pin

# Front Left
IN3 = 23
IN4 = 24
ENB = 18

# Back Left
IN5 = 20
IN6 = 21
ENC = 19

# Back Right
IN7 = 8
IN8 = 7
END = 12

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Suppress warnings

# Setup pins
for pin in [IN1, IN2, ENA, IN3, IN4, ENB, IN5, IN6, ENC, IN7, IN8, END]:
    GPIO.setup(pin, GPIO.OUT)

# PWM Setup (100 Hz frequency)
pwm_a = GPIO.PWM(ENA, 100)  # Front Right
pwm_b = GPIO.PWM(ENB, 100)  # Front Left
pwm_c = GPIO.PWM(ENC, 100)  # Back Left
pwm_d = GPIO.PWM(END, 100)  # Back Right

pwm_a.start(0)
pwm_b.start(0)
pwm_c.start(0)
pwm_d.start(0)

def set_motor_speed(pwm, in1, in2, speed):
    """Set motor direction and speed."""
    if speed >= 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    pwm.ChangeDutyCycle(min(abs(speed), 100))  # Clamp to 0-100

def stop_motors():
    """Stop all motors."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN5, GPIO.LOW)
    GPIO.output(IN6, GPIO.LOW)
    GPIO.output(IN7, GPIO.LOW)
    GPIO.output(IN8, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    pwm_c.ChangeDutyCycle(0)
    pwm_d.ChangeDutyCycle(0)

def callback(data):
    linear_x = data.linear.x    # Forward/backward speed (m/s)
    angular_z = data.angular.z  # Angular speed (rad/s)

    # Log incoming commands for debugging
    rospy.loginfo(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")

    # Scale factors (adjust based on your robot's max speed)
    max_linear = 0.5   # Max linear speed (m/s)
    max_angular = 1.0  # Max angular speed (rad/s)
    max_pwm = 100      # Max PWM duty cycle

    # Normalize velocities to PWM range
    linear_pwm = (linear_x / max_linear) * max_pwm
    angular_pwm = (angular_z / max_angular) * max_pwm

    # Calculate left and right side speeds (differential drive)
    left_speed = linear_pwm - angular_pwm  # Left motors
    right_speed = linear_pwm + angular_pwm  # Right motors

    # Clamp speeds to [-100, 100]
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

    # Apply to motors
    if abs(left_speed) < 5 and abs(right_speed) < 5:  # Dead zone to prevent jitter
        stop_motors()
    else:
        set_motor_speed(pwm_b, IN3, IN4, left_speed)  # Front Left
        set_motor_speed(pwm_c, IN5, IN6, left_speed)  # Back Left
        set_motor_speed(pwm_a, IN1, IN2, right_speed)  # Front Right
        set_motor_speed(pwm_d, IN7, IN8, right_speed)  # Back Right

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
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        stop_motors()
        GPIO.cleanup()