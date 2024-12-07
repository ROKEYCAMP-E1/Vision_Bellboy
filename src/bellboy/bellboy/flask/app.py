from flask import Flask, render_template, request, redirect, url_for, session, Response
import sqlite3

app = Flask(__name__)
app.secret_key = 'your_secret_key'

# 테스트용 아이디, 비밀번호
USER_ID = 'sy'
PASSWORD = '1234'

# 전역 변수로 최신 프레임 저장
top_video_frame = None

# Database connection function
def get_db_connection():
    conn = sqlite3.connect('system.db')
    conn.row_factory = sqlite3.Row
    return conn

# Routes
@app.route('/')
def home():
    return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        # Retrieve form data
        user_id = request.form['user_id']
        password = request.form['password']

        # DB 설계 후 통합
        '''
        conn = get_db_connection()
        user = conn.execute('SELECT * FROM users WHERE user_id = ? AND password = ?', (user_id, password)).fetchone()
        conn.close()

        if user:
            session['user_id'] = user['user_id']
            return redirect(url_for('system_monitor'))
        else:
            return render_template('login.html', error="Invalid credentials")
            '''
        # Hardcoded login validation
        if user_id == USER_ID and password == PASSWORD:
            session['user_id'] = user_id
            return redirect(url_for('system_monitor'))
        else:
            return render_template('login.html', error="Invalid credentials")

    return render_template('login.html')

@app.route('/system_monitor')
def system_monitor():
    if 'user_id' not in session:
        return redirect(url_for('login'))

    return render_template('system_monitor.html')

@app.route('/db_search')
def db_search():
    if 'user_id' not in session:
        return redirect(url_for('login'))

    conn = get_db_connection()
    # Placeholder for actual query
    data = conn.execute('SELECT * FROM your_table').fetchall()
    conn.close()

    return render_template('db_search.html', data=data)

@app.route('/logout')
def logout():
    session.pop('user_id', None)
    return redirect(url_for('login'))


# 영상 업데이트 라우트
@app.route('/top_video', methods=['POST'])
def update_top_video():
    global top_video_frame
    top_video_frame = request.data  # 받은 프레임 저장
    return '', 200

@app.route('/stream/top_video')
def stream_top_video():
    return Response(generate_video_feed(top_video_frame), mimetype='multipart/x-mixed-replace; boundary=frame')

# 영상 스트리밍 라우트
def generate_video_feed(frame_source):
    while True:
        if frame_source is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_source + b'\r\n')
        else:
            yield (b'--frame\r\n'
                   b'Content-Type: text/plain\r\n\r\nNo frame\r\n')

# Templates (to be added as HTML files in a "templates" directory):
# 1. login.html: Login form with fields for user_id and password.
# 2. system_monitor.html: Placeholder for AMR video, top-view video, and task status notifications.
# 3. db_search.html: Placeholder for displaying database search results.

if __name__ == '__main__':
    app.run(debug=True)
