import cv2
import rclpy
from rclpy.node import Node
from multiprocessing import Process, Queue
from ultralytics import YOLO


class TopViewNode(Node):
    def __init__(self, frame_queue):
        super().__init__('topview_node')
        self.frame_counter = 0  # Initialize the frame_counter attribute
        self.frame_queue = frame_queue

        # USB 카메라 연결
        self.cap = cv2.VideoCapture('/dev/video0')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            exit()

        # 타이머 설정 (10 FPS)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.frame_counter += 1
        if self.frame_counter % 2 != 0:  # 매 2번째 프레임만 처리
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame.")
            return
        if self.frame_queue.full():
            _ = self.frame_queue.get()  # 가장 오래된 데이터 삭제
        self.frame_queue.put(frame)
        self.get_logger().info("Frame added to queue.")

        if self.frame_queue.full():
            self.get_logger().warn("Frame queue is full. Dropping frame.")


def yolo_process(frame_queue, result_queue):
    """YOLO 추론 멀티프로세스"""
    model = YOLO("src/bellboy/bellboy/topview_camera/topview_model_v11.pt")
    print("YOLO 모델 로드 완료.")

    while True:
        if frame_queue.empty():
            continue  # 큐가 비어 있으면 대기
        frame = frame_queue.get()
        print("YOLO: 프레임 가져오기 완료.")


        # YOLO 추론
        results = model(frame)
        print("YOLO: 추론 완료.")

        annotated_frame = results[0].plot()
        result_queue.put(annotated_frame)
        print("YOLO: 결과 큐에 추가 완료.")


def http_process(result_queue):
    """HTTP 요청 멀티프로세스"""
    import requests
    server_url = "http://127.0.0.1:5000/topview_cam"

    while True:
        if result_queue.empty():
            continue  # 큐가 비어 있으면 대기
        print("HTTP: result_queue에서 프레임 가져오기.")
        frame = result_queue.get()

        # JPEG로 인코딩
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        if not success:
            print("HTTP: 프레임 인코딩 실패.")
            continue

        try:
            response = requests.post(
                server_url,
                files={"frame": encoded_image.tobytes()}
            )
            if response.status_code == 200:
                print("HTTP: 서버로 프레임 전송 성공.")
            else:
                print(f"HTTP: 서버 응답 오류 {response.status_code}")
        except Exception as e:
            print(f"HTTP 요청 실패: {e}")


def main(args=None):
    rclpy.init(args=args)

    # 큐 생성
    frame_queue = Queue(maxsize=200)
    result_queue = Queue(maxsize=200)

    # YOLO 프로세스 시작
    yolo_p = Process(target=yolo_process, args=(frame_queue, result_queue), daemon=True)
    yolo_p.start()

    # HTTP 프로세스 시작
    http_p = Process(target=http_process, args=(result_queue,), daemon=True)
    http_p.start()

    # ROS 노드 실행
    topview_cam = TopViewNode(frame_queue)
    try:
        rclpy.spin(topview_cam)
    finally:
        topview_cam.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
