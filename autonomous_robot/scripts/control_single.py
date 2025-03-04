#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class DCMotorController:
    def __init__(self):
        rospy.init_node('dc_motor_controller', anonymous=True)

        # GPIO Pin Configuration
        self.motor_pwm_pin = 18  # PWM pin for motor speed control (BCM mode)
        self.motor_dir_pin1 = 23  # Direction pin 1 (optional, for H-bridge control)
        self.motor_dir_pin2 = 24  # Direction pin 2 (optional, for H-bridge control)

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setup(self.motor_pwm_pin, GPIO.OUT)
        GPIO.setup(self.motor_dir_pin1, GPIO.OUT)
        GPIO.setup(self.motor_dir_pin2, GPIO.OUT)

        # Initialize PWM
        self.pwm = GPIO.PWM(self.motor_pwm_pin, 1000)  # 1000 Hz frequency
        self.pwm.start(0)  # Start PWM with 0% duty cycle

        # Subscribe to /cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.motor_speed = 0.0

    def cmd_vel_callback(self, data):
        # Map linear.x (from -1 to 1) to motor speed (0 to 100)
        self.motor_speed = abs(data.linear.x) * 100  # Scale to 0-100%
        direction = data.linear.x > 0  # True for forward, False for backward

        # Set motor direction
        if direction:
            GPIO.output(self.motor_dir_pin1, GPIO.HIGH)
            GPIO.output(self.motor_dir_pin2, GPIO.LOW)
        else:
            GPIO.output(self.motor_dir_pin1, GPIO.LOW)
            GPIO.output(self.motor_dir_pin2, GPIO.HIGH)

        # Set motor speed using PWM
        self.pwm.ChangeDutyCycle(self.motor_speed)
        rospy.loginfo(f"Motor Speed: {self.motor_speed}%, Direction: {'Forward' if direction else 'Backward'}")

    def cleanup(self):
        # Stop PWM and cleanup GPIO
        self.pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("GPIO cleaned up.")

    def run(self):
        rospy.on_shutdown(self.cleanup)
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = DCMotorController()
        controller.run()
    except rospy.ROSInterruptException:
        controller.cleanup()
