#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
import tf
from math import sin, cos

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

class MotorController:
    def __init__(self):
        # ROS Setup
        rospy.init_node('motor_control', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("/cmd_vel", Twist, self.callback)

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [IN1, IN2, ENA, IN3, IN4, ENB, IN5, IN6, ENC, IN7, IN8, END]:
            GPIO.setup(pin, GPIO.OUT)

        # PWM Setup (100 Hz frequency)
        self.pwm_a = GPIO.PWM(ENA, 100)  # Front Right
        self.pwm_b = GPIO.PWM(ENB, 100)  # Front Left
        self.pwm_c = GPIO.PWM(ENC, 100)  # Back Left
        self.pwm_d = GPIO.PWM(END, 100)  # Back Right
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        self.pwm_c.start(0)
        self.pwm_d.start(0)

        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = rospy.Time.now()

    def set_motor_speed(self, pwm, in1, in2, speed):
        """Set motor direction and speed."""
        if speed >= 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(min(abs(speed), 100))  # Clamp to 0-100

    def stop_motors(self):
        """Stop all motors."""
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN5, GPIO.LOW)
        GPIO.output(IN6, GPIO.LOW)
        GPIO.output(IN7, GPIO.LOW)
        GPIO.output(IN8, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.pwm_c.ChangeDutyCycle(0)
        self.pwm_d.ChangeDutyCycle(0)

    def callback(self, data):
        # Extract velocities
        self.vx = data.linear.x    # Forward/backward speed (m/s)
        self.vth = data.angular.z  # Angular speed (rad/s)
        rospy.loginfo(f"Received cmd_vel: linear_x={self.vx}, angular_z={self.vth}")

        # Update odometry
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        delta_x = self.vx * dt * cos(self.th)
        delta_y = self.vx * dt * sin(self.th)
        delta_th = self.vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.last_time = current_time

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.th / 2)
        odom.pose.pose.orientation.w = cos(self.th / 2)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

        # Broadcast TF
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.th),
            current_time,
            "base_link",
            "odom"
        )

        # Motor control
        max_linear = 0.5   # Max linear speed (m/s)
        max_angular = 1.0  # Max angular speed (rad/s)
        max_pwm = 100      # Max PWM duty cycle

        linear_pwm = (self.vx / max_linear) * max_pwm
        angular_pwm = (self.vth / max_angular) * max_pwm

        left_speed = linear_pwm + angular_pwm  # Left motors
        right_speed = linear_pwm - angular_pwm  # Right motors

        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        if abs(left_speed) < 5 and abs(right_speed) < 5:
            self.stop_motors()
        else:
            self.set_motor_speed(self.pwm_b, IN3, IN4, left_speed)  # Front Left
            self.set_motor_speed(self.pwm_c, IN5, IN6, left_speed)  # Back Left
            self.set_motor_speed(self.pwm_a, IN1, IN2, right_speed)  # Front Right
            self.set_motor_speed(self.pwm_d, IN7, IN8, right_speed)  # Back Right

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        controller.stop_motors()
        GPIO.cleanup()
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        controller.stop_motors()
        GPIO.cleanup()