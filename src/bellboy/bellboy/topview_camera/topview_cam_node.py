# topview_cam_node.py

import cv2
import rclpy
import requests
from rclpy.node import Node
from threading import Thread, Lock
from queue import Queue
from ultralytics import YOLO


class TopViewNode(Node):
    def __init__(self, frame_queue):
        super().__init__('topview_node')

        # USB 카메라 연결
        self.cap = cv2.VideoCapture('/dev/video2')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            exit()

        self.frame_queue = frame_queue
        self.lock = Lock()

        # 타이머 설정 (10 FPS)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame.")
            return

        with self.lock:
            if not self.frame_queue.full():
                self.frame_queue.put(frame)


def yolo_thread(frame_queue, result_queue):
    """YOLO 추론 스레드"""
    model = YOLO("src/bellboy/bellboy/topview_camera/topview_model_v11.pt")

    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            resized_frame = cv2.resize(frame, (640, 360))
            # YOLO 추론
            results = model(resized_frame, conf=0.65)
            annotated_frame = results[0].plot()
            result_queue.put(annotated_frame)


def http_thread(result_queue):
    """HTTP 요청 스레드"""
    server_url = "http://127.0.0.1:5000/topview_cam"

    while True:
        if not result_queue.empty():
            frame = result_queue.get()

            # JPEG로 인코딩
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            if not success:
                print("Failed to encode frame.")
                continue

            try:
                response = requests.post(
                    server_url,
                    files={"frame": encoded_image.tobytes()}
                )
                if response.status_code != 200:
                    print(f"Server response error: {response.status_code}")
            except Exception as e:
                print(f"HTTP request failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    # 큐 생성
    frame_queue = Queue(maxsize=10)
    result_queue = Queue(maxsize=10)

    # YOLO 스레드 시작
    yolo_t = Thread(target=yolo_thread, args=(frame_queue, result_queue), daemon=True)
    yolo_t.start()

    # HTTP 스레드 시작
    http_t = Thread(target=http_thread, args=(result_queue,), daemon=True)
    http_t.start()

    # ROS 노드 실행
    topview_cam = TopViewNode(frame_queue)
    try:
        rclpy.spin(topview_cam)
    finally:
        topview_cam.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
