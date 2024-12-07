# manager_interface.py

from flask import Flask, render_template, Response, request
import cv2
import numpy as np
import threading
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)

# 전역 변수 및 락 설정
latest_frame = None
frame_lock = threading.Lock()

@app.route('/')
def index():
    """HTML 페이지 렌더링"""
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

    # WebSocket을 통해 클라이언트로 전송
    if latest_frame is not None:
        _, buffer = cv2.imencode('.jpg', latest_frame)
        socketio.emit('frame', buffer.tobytes())
    return "Frame received", 200

@socketio.on('connect')
def handle_connect():
    """클라이언트가 연결되었을 때"""
    print("Client connected.")

@socketio.on('disconnect')
def handle_disconnect():
    """클라이언트가 연결이 끊겼을 때"""
    print("Client disconnected.")


if __name__ == "__main__":
    app.run(debug=True)
