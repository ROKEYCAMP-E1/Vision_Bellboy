# amr_node.py
# jetson nano에서 돌아가야 하는 코드
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class AMRNode(Node):
    def __init__(self):
        super().__init__('amr_node')
        self.frame_counter = 0  # Initialize the frame_counter attribute

        # ROS 2 퍼블리셔 생성
        self.publisher_ = self.create_publisher(CompressedImage, 'amr_video', 10)

        # OpenCV와 ROS 간 변환 도구
        self.bridge = CvBridge()

        # YOLO 모델 로드
        self.model = YOLO("AMR_camera/AMR_epoch90.pt")

        # USB 카메라 연결
        self.cap = cv2.VideoCapture('/dev/video2')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            exit()

        # 주기적으로 실행할 타이머 생성 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.frame_counter += 1
        if self.frame_counter % 2 != 0:  # 매 2번째 프레임만 처리
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame.")
            return

        # YOLO 모델로 추적 수행
        results = self.model.track(source=frame, tracker="botsort.yaml", conf=0.9)

        # 추적 결과 표시
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
    amr_node = AMRNode()

    try:
        rclpy.spin(amr_node)
    except KeyboardInterrupt:
        amr_node.get_logger().info("Node interrupted by user.")
    finally:
        amr_node.cap.release()
        amr_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
