# testui_node.py

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
                             QStackedWidget, QTextEdit, QTableWidget, QTableWidgetItem)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class VideoThread(QThread):
    frame_received = pyqtSignal(str, np.ndarray)  # 토픽 이름과 프레임을 전달

    def __init__(self, topic_name, ros_node):
        super().__init__()
        self.topic_name = topic_name
        self.ros_node = ros_node
        self.bridge = CvBridge()
        self.running = True

        # ROS2 구독자 설정
        self.subscription = self.ros_node.create_subscription(
            CompressedImage,
            self.topic_name,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """ROS2 콜백: 메시지를 디코딩하고 신호를 통해 전달"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.frame_received.emit(self.topic_name, frame)  # PyQt 신호로 전달
        except Exception as e:
            self.ros_node.get_logger().error(f"Failed to process frame from {self.topic_name}: {e}")

    def run(self):
        """ROS2 스레드 실행"""
        while self.running:
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def stop(self):
        """스레드 종료"""
        self.running = False

class TestUINode(QWidget):
    def __init__(self, ros_node, parent_widget):
        super().__init__()
        self.ros_node = ros_node
        self.parent_widget = parent_widget  # QStackedWidget 참조
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
        # 버튼 클릭 이벤트에 대한 처리
        self.button_db.clicked.connect(self.go_to_db)

        # 레이아웃 설정
        layout = QVBoxLayout()
        video_layout = QHBoxLayout()
        video_layout.addWidget(self.label_top_video)
        video_layout.addWidget(self.label_amr_video)

        layout.addLayout(video_layout)
        layout.addWidget(QLabel("Notification Panel:"))
        layout.addWidget(self.notification_panel)
        layout.addWidget(self.button_db)
        self.setLayout(layout)

        # ROS2와 PyQt5의 멀티스레딩
        self.top_video_thread = VideoThread('top_video', self.ros_node)
        self.amr_video_thread = VideoThread('amr_video', self.ros_node)

        # 스레드와 UI 연결
        self.top_video_thread.frame_received.connect(self.update_frame)
        self.amr_video_thread.frame_received.connect(self.update_frame)

        # 스레드 시작
        self.top_video_thread.start()
        self.amr_video_thread.start()

    def go_to_db(self):
        self.parent_widget.setCurrentIndex(2)  # DBScreen 화면으로 전환

    def update_frame(self, topic_name, frame):
        """QLabel에 프레임 업데이트"""
        try:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qimage)
            scaled_pixmap = pixmap.scaled(640, 480, Qt.KeepAspectRatio)

            if topic_name == 'top_video':
                self.label_top_video.setPixmap(scaled_pixmap)
            elif topic_name == 'amr_video':
                self.label_amr_video.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"Failed to update frame for {topic_name}: {e}")

    def closeEvent(self, event):
        """창 닫힐 때 스레드 종료"""
        self.top_video_thread.stop()
        self.amr_video_thread.stop()
        self.top_video_thread.wait()
        self.amr_video_thread.wait()
        event.accept()

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
        self.ros_node = Node('ui_node')

        self.setWindowTitle("ROS 2 UI")
        self.stacked_widget = QStackedWidget()

        # 로그인, 모니터, DB 화면 추가
        self.login_screen = LoginScreen(self.stacked_widget)
        self.monitor_screen = TestUINode(self.ros_node, self.stacked_widget)  # QStackedWidget 전달
        self.db_screen = DBScreen(self.stacked_widget)  # DB 화면 생성 시 부모 연결

        self.stacked_widget.addWidget(self.login_screen)
        self.stacked_widget.addWidget(self.monitor_screen)
        self.stacked_widget.addWidget(self.db_screen)

        layout = QVBoxLayout()
        layout.addWidget(self.stacked_widget)
        self.setLayout(layout)

def main():
    # ROS2 초기화
    rclpy.init()

    app = QApplication(sys.argv)
    test_ui_node = MainApp()
    test_ui_node.show()

    
    # ROS 2 실행을 위한 타이머 설정
    def ros_spin():
        # TestUINode 내부 노드를 올바르게 참
        rclpy.spin_once(test_ui_node.ros_node, timeout_sec=0.01)

    # ROS 2 실행을 위한 타이머 설정
    timer = app.timer = app.startTimer(100)
    app.timerEvent = lambda e: ros_spin()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()