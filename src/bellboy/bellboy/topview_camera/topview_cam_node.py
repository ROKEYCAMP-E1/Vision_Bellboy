# topview_cam_node.py

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO



class topview_node(Node):
    def __init__(self):
        super().__init__('topview_node')
        self.publisher_ = self.create_publisher(CompressedImage, 'top_video', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  

        # OpenCV와 ROS 간 이미지를 변환할 브리지
        self.bridge = CvBridge()

        # YOLO 모델 로드
        self.model = YOLO("src/bellboy/bellboy/topview_camera/topview_model_v11.pt")

        # USB 카메라 연결
        self.cap = cv2.VideoCapture('/dev/video0') # 카메라 usb번호 입력해주세요
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame.")
            return

        # YOLO 모델로 탐지 수행
        results = self.model(frame, conf=0.65)

        # 탐지 결과 표시 (annotated_frame)
        annotated_frame = results[0].plot()

        # OpenCV 이미지를 JPEG로 압축
        success, encoded_image = cv2.imencode('.jpg', annotated_frame)
        if not success:
            self.get_logger().error("Failed to encode frame.")
            return

        # CompressedImage 메시지 생성
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.format = "jpeg"
        compressed_image.data = encoded_image.tobytes()


        # ROS 메시지 퍼블리싱
        self.publisher_.publish(compressed_image)

def main(args=None):
    rclpy.init(args=args)
    topview_cam = topview_node()
    rclpy.spin(topview_cam)
    topview_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()