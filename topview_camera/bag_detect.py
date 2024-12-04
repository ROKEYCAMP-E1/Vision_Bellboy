import cv2
from ultralytics import YOLO

# YOLO 모델 로드 (학습된 모델 경로 지정)
model = YOLO("topview_camera/topview_model_v11.pt")

# USB 카메라 연결 (/dev/video1)
cap = cv2.VideoCapture('/dev/video2')

# 카메라 연결 확인
if not cap.isOpened():
    print("Error: Cannot open camera.")
    exit()

# 실시간 추적 루프
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # YOLO 모델로 추적 수행
    results = model(source=frame, tracker="botsort.yaml", conf=0.7)  # Botsort 추적기 사용

    # 추적 결과 표시
    annotated_frame = results[0].plot()  # 추적 결과 시각화된 프레임

    # 화면에 표시
    cv2.imshow("YOLO Real-Time Tracking", annotated_frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()
