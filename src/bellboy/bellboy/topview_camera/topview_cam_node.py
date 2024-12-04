# topview_cam_node.py

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO



class topview_node(Node):
    def __init__(self):
        super().__init__('topview_node')
        self.publisher_ = self.create_publisher(Image, 'top_video', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  

        # OpenCV와 ROS 간 이미지를 변환할 브리지
        self.bridge = CvBridge()

        # YOLO 모델 로드
        self.model = YOLO("src/bellboy/bellboy/topview_camera/topview_model_v11.pt")

        # USB 카메라 연결
        self.cap = cv2.VideoCapture('/dev/video2') # 카메라 usb번호 입력해주세요
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame.")
            return

        # YOLO 모델로 탐지 수행
        results = self.model(frame, conf=0.7)

        # 탐지 결과 표시 (annotated_frame)
        annotated_frame = results[0].plot()

        # OpenCV 이미지를 ROS 2 메시지로 변환
        ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        # ROS 메시지 퍼블리싱
        self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    topview_cam = topview_node()
    rclpy.spin(topview_cam)
    topview_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()