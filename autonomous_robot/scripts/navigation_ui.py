#!/usr/bin/env python3
import sys
import math
import threading
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QPushButton,
    QLabel, QVBoxLayout, QTextEdit
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist, Point
from actionlib_msgs.msg import GoalStatusArray

class MapWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(400, 400)
        self.robot_pose = Point(0, 0, 0)  # x, y, z (z unused for 2D)
        self.laser_points = []  # List of (x, y) points from laser scan
        self.scale = 20  # Pixels per meter for rendering

    def update_pose(self, pose):
        self.robot_pose = pose
        self.update()  # Trigger repaint

    def update_laser(self, points):
        self.laser_points = points
        self.update()  # Trigger repaint

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw grid (10x10 meter map, 1m per grid cell)
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        for i in range(0, self.width(), int(self.scale)):
            painter.drawLine(i, 0, i, self.height())
        for i in range(0, self.height(), int(self.scale)):
            painter.drawLine(0, i, self.width(), i)

        # Translate to center of widget
        painter.translate(self.width() / 2, self.height() / 2)

        # Draw laser scan points
        painter.setPen(QPen(QColor(255, 0, 0), 2))
        for x, y in self.laser_points:
            painter.drawPoint(int(x * self.scale), int(-y * self.scale))  # Flip y for screen coords

        # Draw robot
        painter.setBrush(QColor(0, 128, 255))
        painter.drawEllipse(int(self.robot_pose.x * self.scale - 10),
                          int(-self.robot_pose.y * self.scale - 10), 20, 20)

class RobotNavigationUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Navigation Control")
        self.setGeometry(100, 100, 800, 600)
        self.auto_mode = False  # Track manual/auto mode

        # Initialize ROS node
        try:
            rospy.init_node('robot_navigation_ui', anonymous=True)
        except rospy.ROSException as e:
            print(f"ROS init failed: {e}")

        # ROS publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        grid_layout = QGridLayout(main_widget)

        # Map display
        self.map_widget = MapWidget()
        grid_layout.addWidget(self.map_widget, 0, 0, 3, 3)

        # Control panel
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)

        # Navigation buttons
        nav_buttons = QWidget()
        nav_layout = QGridLayout(nav_buttons)
        self.up_button = QPushButton("↑ Forward")
        self.down_button = QPushButton("↓ Backward")
        self.left_button = QPushButton("← Left")
        self.right_button = QPushButton("→ Right")
        self.stop_button = QPushButton("Stop")
        
        nav_layout.addWidget(self.up_button, 0, 1)
        nav_layout.addWidget(self.left_button, 1, 0)
        nav_layout.addWidget(self.stop_button, 1, 1)
        nav_layout.addWidget(self.right_button, 1, 2)
        nav_layout.addWidget(self.down_button, 2, 1)

        control_layout.addWidget(nav_buttons)

        # Auto mode toggle
        self.auto_mode_button = QPushButton("Switch to Auto Mode")
        self.auto_mode_button.setStyleSheet("background-color: green; color: white;")
        control_layout.addWidget(self.auto_mode_button)

        # Emergency stop
        self.emergency_stop = QPushButton("Emergency Stop")
        self.emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        control_layout.addWidget(self.emergency_stop)

        # Status display
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        self.position_label = QLabel("Position: (0, 0)")
        self.battery_label = QLabel("Battery: 100%")
        self.mapping_label = QLabel("Mapping: Idle")
        self.planning_label = QLabel("Planning: Idle")
        status_layout.addWidget(self.position_label)
        status_layout.addWidget(self.battery_label)
        status_layout.addWidget(self.mapping_label)
        status_layout.addWidget(self.planning_label)
        control_layout.addWidget(status_widget)

        # Log display
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setPlaceholderText("Navigation logs...")
        control_layout.addWidget(self.log_display)

        grid_layout.addWidget(control_panel, 0, 3, 3, 1)

        # Connect buttons to functions
        self.up_button.clicked.connect(self.move_forward)
        self.down_button.clicked.connect(self.move_backward)
        self.left_button.clicked.connect(self.turn_left)
        self.right_button.clicked.connect(self.turn_right)
        self.stop_button.clicked.connect(self.stop_robot)
        self.emergency_stop.clicked.connect(self.emergency_stop_robot)
        self.auto_mode_button.clicked.connect(self.toggle_auto_mode)

        # ROS subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.battery_sub = rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        self.nav_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.nav_status_callback)

        # Update navigation button states
        self.update_nav_buttons()

        # Timer to keep UI responsive
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # Update every 100ms

    def odom_callback(self, msg):
        # Update robot position from odometry
        pose = msg.pose.pose.position
        self.map_widget.update_pose(pose)
        self.position_label.setText(f"Position: ({pose.x:.2f}, {pose.y:.2f})")

    def scan_callback(self, msg):
        # Convert laser scan to (x, y) points
        points = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_max and r > msg.range_min:
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
        self.map_widget.update_laser(points)

    def battery_callback(self, msg):
        # Update battery status
        percentage = msg.percentage if hasattr(msg, 'percentage') else 100.0
        self.battery_label.setText(f"Battery: {percentage:.1f}%")

    def nav_status_callback(self, msg):
        # Update mapping and planning status based on navigation stack
        if not msg.status_list:
            self.planning_label.setText("Planning: Idle")
        else:
            status = msg.status_list[-1].status
            if status == 1:  # Active
                self.planning_label.setText("Planning: Path Finding")
            elif status == 3:  # Succeeded
                self.planning_label.setText("Planning: Path Found")
            else:  # Failed or other states
                self.planning_label.setText("Planning: Failed")
        # Mapping status (placeholder, assumes SLAM is active in auto mode)
        self.mapping_label.setText("Mapping: In Progress" if self.auto_mode else "Mapping: Idle")

    def toggle_auto_mode(self):
        self.auto_mode = not self.auto_mode
        if self.auto_mode:
            self.auto_mode_button.setText("Switch to Manual Mode")
            self.auto_mode_button.setStyleSheet("background-color: orange; color: white;")
            self.log_display.append("Switched to Auto Mode")
            # Placeholder for enabling autonomous navigation (e.g., start move_base)
        else:
            self.auto_mode_button.setText("Switch to Auto Mode")
            self.auto_mode_button.setStyleSheet("background-color: green; color: white;")
            self.log_display.append("Switched to Manual Mode")
            # Stop any autonomous navigation
            self.stop_robot()
        self.update_nav_buttons()

    def update_nav_buttons(self):
        # Disable navigation buttons in auto mode
        state = not self.auto_mode
        self.up_button.setEnabled(state)
        self.down_button.setEnabled(state)
        self.left_button.setEnabled(state)
        self.right_button.setEnabled(state)
        self.stop_button.setEnabled(state)

    def move_forward(self):
        if not self.auto_mode:
            twist = Twist()
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
            self.cmd_vel_pub.publish(twist)
            self.log_display.append("Moving forward...")

    def move_backward(self):
        if not self.auto_mode:
            twist = Twist()
            twist.linear.x = -0.5  # Move backward at 0.5 m/s
            self.cmd_vel_pub.publish(twist)
            self.log_display.append("Moving backward...")

    def turn_left(self):
        if not self.auto_mode:
            twist = Twist()
            twist.angular.z = 0.5  # Turn left at 0.5 rad/s
            self.cmd_vel_pub.publish(twist)
            self.log_display.append("Turning left...")

    def turn_right(self):
        if not self.auto_mode:
            twist = Twist()
            twist.angular.z = -0.5  # Turn right at 0.5 rad/s
            self.cmd_vel_pub.publish(twist)
            self.log_display.append("Turning right...")

    def stop_robot(self):
        twist = Twist()  # Zero velocity
        self.cmd_vel_pub.publish(twist)
        self.log_display.append("Robot stopped.")

    def emergency_stop_robot(self):
        self.stop_robot()
        self.log_display.append("Emergency stop activated!")
        if self.auto_mode:
            self.toggle_auto_mode()

    def update_ui(self):
        # Placeholder for additional UI updates
        pass

    def closeEvent(self, event):
        # Clean up ROS subscribers and publishers
        self.odom_sub.unregister()
        self.scan_sub.unregister()
        self.battery_sub.unregister()
        self.nav_status_sub.unregister()
        self.cmd_vel_pub.unregister()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Run ROS in a separate thread
    def ros_spin():
        rospy.spin()
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    window = RobotNavigationUI()
    window.show()
    sys.exit(app.exec_())
