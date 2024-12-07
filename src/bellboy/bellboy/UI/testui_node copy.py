# testui_node.py

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
                             QStackedWidget, QTextEdit, QTableWidget, QTableWidgetItem)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class VideoSubscriber(Node):
    def __init__(self, topic_name, update_callback):
        super().__init__('video_subscriber_' + topic_name)
        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,  # 구독할 토픽 이름
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.update_callback = update_callback  # UI 업데이트 콜백 함수

    def listener_callback(self, msg):
        # CompressedImage 데이터를 OpenCV 이미지로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 디코딩된 이미지를 UI에 업데이트
        self.update_callback(frame)


class LoginScreen(QWidget):
    def __init__(self, parent_widget):
        super().__init__()
        self.parent_widget = parent_widget

        layout = QVBoxLayout()

        self.label_id = QLabel("ID:")
        self.input_id = QLineEdit()

        self.label_password = QLabel("Password:")
        self.input_password = QLineEdit()
        self.input_password.setEchoMode(QLineEdit.Password)

        self.button_login = QPushButton("Login")
        self.button_login.clicked.connect(self.verify_login)

        layout.addWidget(self.label_id)
        layout.addWidget(self.input_id)
        layout.addWidget(self.label_password)
        layout.addWidget(self.input_password)
        layout.addWidget(self.button_login)

        self.setLayout(layout)

    def verify_login(self):
        hardcoded_users = {
            "admin": "sy",
            "user1": "sy",
        }
        if self.input_id.text() in hardcoded_users and self.input_password.text() == hardcoded_users[self.input_id.text()]:
            self.parent_widget.setCurrentIndex(1)  # Go to Monitor Screen
        else:
            self.label_password.setText("Password: (Invalid)")

class TestUINode(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Video Streams")
        self.resize(1600, 600)

        # QLabel을 사용하여 두 개의 영상을 표시
        self.label_top_video = QLabel("Waiting for top_video...")
        self.label_amr_video = QLabel("Waiting for amr_video...")
        self.label_top_video.setAlignment(Qt.AlignCenter)
        self.label_amr_video.setAlignment(Qt.AlignCenter)

        self.notification_panel = QTextEdit()
        self.notification_panel.setReadOnly(True)
        self.notification_panel.setText("Current Task: None")
        
        self.button_db = QPushButton("Go to DB Page")
        self.button_db.clicked.connect(lambda: self.parent_widget.setCurrentIndex(2))

        # 레이아웃 설정 (가로로 두 영상 나란히 배치)
        layout = QVBoxLayout()
        video_layout = QHBoxLayout()
        video_layout.addWidget(self.label_top_video)
        video_layout.addWidget(self.label_amr_video)

        layout.addLayout(video_layout)
        layout.addWidget(QLabel("Notification Panel:"))
        layout.addWidget(self.notification_panel)
        layout.addWidget(self.button_db)
        self.setLayout(layout)

        self.node_top_video = VideoSubscriber('top_video', self.update_top_video)
        self.node_amr_video = VideoSubscriber('amr_video', self.update_amr_video)

    def update_top_video(self, frame):
        # OpenCV 이미지를 PyQt5 QLabel로 변환 (top_video)
        self.update_label(self.label_top_video, frame)

    def update_amr_video(self, frame):
        # OpenCV 이미지를 PyQt5 QLabel로 변환 (amr_video)
        self.update_label(self.label_amr_video, frame)

    def update_label(self, label, frame):
        # OpenCV 이미지를 PyQt5 QLabel로 변환
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qimage)

        # QLabel 크기에 맞춰 QPixmap 크기를 조정
        scaled_pixmap = pixmap.scaled(label.width(), label.height(), Qt.KeepAspectRatio)
        label.setPixmap(scaled_pixmap)

    def closeEvent(self, event):
        # 창 닫힐 때 ROS 2 노드 종료
        self.node_top_video.destroy_node()
        self.node_amr_video.destroy_node()
        rclpy.shutdown()
        event.accept()

class DBScreen(QWidget):
    def __init__(self, parent_widget):
        super().__init__()
        self.parent_widget = parent_widget

        layout = QVBoxLayout()

        self.table = QTableWidget()
        self.table.setRowCount(0)
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["ID", "Task", "Timestamp"])

        self.button_back = QPushButton("Back to Monitor Screen")
        self.button_back.clicked.connect(lambda: self.parent_widget.setCurrentIndex(1))

        layout.addWidget(self.table)
        layout.addWidget(self.button_back)

        self.setLayout(layout)
        self.load_data()

    def load_data(self):
        data = [
            ("1", "Task A", "2024-12-06 10:00"),
            ("2", "Task B", "2024-12-06 11:00"),
        ]
        self.table.setRowCount(len(data))
        for row_idx, row_data in enumerate(data):
            for col_idx, col_data in enumerate(row_data):
                self.table.setItem(row_idx, col_idx, QTableWidgetItem(col_data))

class MainApp(QWidget):
    def __init__(self):
        super().__init__()

        # ROS2 초기화
        rclpy.init()

        self.setWindowTitle("ROS 2 UI")
        self.stacked_widget = QStackedWidget()

        self.login_screen = LoginScreen(self.stacked_widget)
        self.monitor_screen = TestUINode(self.stacked_widget)
        self.db_screen = DBScreen(self.stacked_widget)

        self.stacked_widget.addWidget(self.login_screen)
        self.stacked_widget.addWidget(self.monitor_screen)
        self.stacked_widget.addWidget(self.db_screen)

        layout = QVBoxLayout()
        layout.addWidget(self.stacked_widget)
        self.setLayout(layout)

def main():
    
    app = QApplication(sys.argv)
    test_ui_node = TestUINode()
    test_ui_node.show()

    # ROS 2 실행을 위한 타이머 설정
    timer = app.timer = app.startTimer(100)
    def ros_spin():
        rclpy.spin_once(test_ui_node.node_top_video, timeout_sec=0.01)
        rclpy.spin_once(test_ui_node.node_amr_video, timeout_sec=0.01)
    app.timerEvent = lambda e: ros_spin()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()