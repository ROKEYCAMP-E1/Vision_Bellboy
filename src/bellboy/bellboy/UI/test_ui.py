# test_ui.py

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class VideoSubscriber(Node):
    def __init__(self, update_callback):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'top_video',  # 송출된 토픽 이름
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.update_callback = update_callback  # 이미지를 업데이트할 콜백 함수

    def listener_callback(self, msg):
        # ROS 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.update_callback(frame)  # 이미지를 UI에 업데이트


class testUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Video Stream")
        self.resize(800, 600)

        # QLabel을 사용하여 영상을 표시
        self.label = QLabel("Waiting for video...")
        self.label.setAlignment(Qt.AlignCenter)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # ROS 2 노드 초기화
        rclpy.init()
        self.node = VideoSubscriber(self.update_image)

    def update_image(self, frame):
        # OpenCV 이미지를 PyQt5 QLabel로 변환
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qimage)

        # QLabel에 Pixmap 업데이트
        self.label.setPixmap(pixmap)

    def closeEvent(self, event):
        # 창 닫힐 때 ROS 2 노드 종료
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    video_ui = testUI()
    video_ui.show()

    # ROS 2 실행을 위한 타이머 설정
    timer = app.timer = app.startTimer(100)
    def ros_spin():
        rclpy.spin_once(video_ui.node, timeout_sec=0.01)
    app.timerEvent = lambda e: ros_spin()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
