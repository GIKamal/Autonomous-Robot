#!/usr/bin/env python3
import rospy # type: ignore
from geometry_msgs.msg import Twist # type: ignore
import RPi.GPIO as GPIO # type: ignore

# GPIO Pin Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor 1 (L298N 1)
IN1 = 17
IN2 = 18
ENA = 22

# Motor 2 (L298N 1)
IN3 = 23
IN4 = 24
ENB = 25

# Motor 3 (L298N 2)
IN5 = 5
IN6 = 6
ENC = 13

# Motor 4 (L298N 2)
IN7 = 19
IN8 = 26
END = 12

# Set up GPIO pins
GPIO.setup([IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8], GPIO.OUT)
GPIO.setup([ENA, ENB, ENC, END], GPIO.OUT)

# PWM Setup
pwm_a = GPIO.PWM(ENA, 1000)  # 1000 Hz frequency
pwm_b = GPIO.PWM(ENB, 1000)
pwm_c = GPIO.PWM(ENC, 1000)
pwm_d = GPIO.PWM(END, 1000)
pwm_a.start(0)
pwm_b.start(0)
pwm_c.start(0)
pwm_d.start(0)

def set_motor_speed(motor, speed):
    """Set speed and direction for a motor."""
    if motor == 1:
        in1, in2, pwm = IN1, IN2, pwm_a
    elif motor == 2:
        in1, in2, pwm = IN3, IN4, pwm_b
    elif motor == 3:
        in1, in2, pwm = IN5, IN6, pwm_c
    elif motor == 4:
        in1, in2, pwm = IN7, IN8, pwm_d
    else:
        return

    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    pwm.ChangeDutyCycle(abs(speed))

def cmd_vel_callback(msg):
    """Convert Twist message to motor speeds."""
    linear_x = msg.linear.x  # Forward/backward velocity
    angular_z = msg.angular.z  # Rotational velocity

    # Adjust these scaling factors as needed
    max_speed = 100  # Maximum PWM duty cycle (0-100)
    speed_scaling = 50  # Scaling factor for linear velocity
    turn_scaling = 30  # Scaling factor for angular velocity

    # Calculate motor speeds
    left_speed = (linear_x * speed_scaling) - (angular_z * turn_scaling)
    right_speed = (linear_x * speed_scaling) + (angular_z * turn_scaling)

    # Limit speeds to the maximum value
    left_speed = max(min(left_speed, max_speed), -max_speed)
    right_speed = max(min(right_speed, max_speed), -max_speed)

    # Set motor speeds
    set_motor_speed(1, left_speed)  # Left front motor
    set_motor_speed(2, right_speed)  # Right front motor
    set_motor_speed(3, left_speed)  # Left rear motor
    set_motor_speed(4, right_speed)  # Right rear motor

def motor_control_node():
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_control_node()
    except rospy.ROSInterruptException:
        GPIO.cleanup()