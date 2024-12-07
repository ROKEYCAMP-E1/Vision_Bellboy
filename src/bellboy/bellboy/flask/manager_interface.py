# manager_interface.py

from flask import Flask, render_template, Response, request
import cv2
import numpy as np
import threading

app = Flask(__name__)

# 전역 변수 및 락 설정
latest_frame = None
frame_lock = threading.Lock()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/topview_cam', methods=['POST'])
def upload_frame():
    """topview_cam_node.py에서 전송된 프레임을 저장"""
    global latest_frame
    if 'frame' not in request.files:
        return "No frame received", 400

    # 프레임 읽기
    file = request.files['frame']
    frame_data = file.read()

    # 최신 프레임 저장
    with frame_lock:
        np_arr = np.frombuffer(frame_data, np.uint8)
        latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return "Frame received", 200

def generate_frames():
    """Flask 스트리밍용 프레임 생성"""
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
            else:
                frame = None

        if frame is not None:
            success, buffer = cv2.imencode('.jpg', frame)
            if success:
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """비디오 피드를 스트리밍"""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(debug=True)
