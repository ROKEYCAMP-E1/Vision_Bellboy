# manager_interface.py

from flask import Flask, render_template, request, redirect, url_for, session
import cv2
import numpy as np
import threading
from flask_socketio import SocketIO

app = Flask(__name__)
app.secret_key = 'your_secret_key'  # 세션 관리를 위한 Secret Key
socketio = SocketIO(app)

# 전역 변수 및 락 설정
latest_frame = None
frame_lock = threading.Lock()

# 로그인 데이터 (임시로 하드코딩)
USERS = {
    'sy': '1',
    'he': '1'
}

@app.route('/')
def login():
    """로그인 페이지 렌더링"""
    if 'user' in session:
        return redirect(url_for('monitoring'))  # 이미 로그인한 경우 모니터링 화면으로
    return render_template('login.html')

@app.route('/login', methods=['POST'])
def do_login():
    """로그인 처리"""
    username = request.form.get('username')
    password = request.form.get('password')

    # 사용자 인증
    if username in USERS and USERS[username] == password:
        session['user'] = username
        return redirect(url_for('monitoring'))
    else:
        error = "Invalid username or password. Please try again."
        return render_template('login.html', error=error)

@app.route('/monitoring')
def monitoring():
    """모니터링 화면 (로그인 필요)"""
    if 'user' not in session:
        return redirect(url_for('login'))  # 로그인하지 않은 경우 로그인 화면으로
    return render_template('monitoring.html', user=session['user'])

@app.route('/statistics')
def statistics():
    """서비스 통계 페이지"""
    if 'user' not in session:
        return redirect(url_for('login'))  # 로그인하지 않은 경우 로그인 화면으로
    return render_template('statistics.html', user=session['user'])


@app.route('/logout')
def logout():
    """로그아웃 처리"""
    session.pop('user', None)
    return redirect(url_for('login'))

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

@app.route('/amr_cam', methods=['POST'])
def upload_amr_frame():
    """interface_node.py에서 전송된 AMR 카메라 프레임을 저장"""
    global amr_frame
    if 'frame' not in request.files:
        return "No frame received", 400

    # 프레임 읽기
    file = request.files['frame']
    frame_data = file.read()

    # 최신 프레임 저장
    with frame_lock:
        np_arr = np.frombuffer(frame_data, np.uint8)
        amr_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # WebSocket을 통해 클라이언트로 전송
    if amr_frame is not None:
        _, buffer = cv2.imencode('.jpg', amr_frame)
        socketio.emit('amr_frame', buffer.tobytes())
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
