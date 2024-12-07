import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
                             QStackedWidget, QTextEdit, QTableWidget, QTableWidgetItem)
from PyQt5.QtCore import Qt

class ROSNode(Node):
    def __init__(self):
        super().__init__('robot_interface_node')
        self.customer_decision_pub = self.create_publisher(Bool, '/CustomerDecision', 10)
        self.service_terminate_pub = self.create_publisher(String, '/ServiceTerminate', 10)

    def publish_customer_decision(self, decision: bool):
        msg = Bool()
        msg.data = decision
        self.customer_decision_pub.publish(msg)
        self.get_logger().info(f"Published to /CustomerDecision: {decision}")

    def publish_service_terminate(self):
        msg = String()
        msg.data = "서비스 정상 종료"
        self.service_terminate_pub.publish(msg)
        self.get_logger().info(f"Published to /ServiceTerminate: 서비스 정상 종료")


class LoginScreen(QWidget):
    def __init__(self, parent_widget):
        super().__init__()
        self.parent_widget = parent_widget

        layout = QVBoxLayout()

        self.label_id = QLabel("ID:")
        self.input_id = QLineEdit()

        self.label_password = QLabel("Password:")
        self.input_password = QLineEdit()
        self.input_password.setEchoMode(QLineEdit.Password)

        self.button_login = QPushButton("Login")
        self.button_login.clicked.connect(self.verify_login)

        layout.addWidget(self.label_id)
        layout.addWidget(self.input_id)
        layout.addWidget(self.label_password)
        layout.addWidget(self.input_password)
        layout.addWidget(self.button_login)

        self.setLayout(layout)

    def verify_login(self):
        # 로그인 검증 (하드코딩된 값)
        hardcoded_id = "admin"
        hardcoded_pw = "1"

        # 로그인 성공 처리
        self.parent_widget.setCurrentIndex(1)  # 서비스 이용 확인 화면으로 전환


class ServiceConfirmScreen(QWidget):
    def __init__(self, parent_widget, ros_node):
        super().__init__()
        self.parent_widget = parent_widget
        self.ros_node = ros_node
        self.setWindowTitle("Service Confirmation")

        layout = QVBoxLayout()

        self.label = QLabel(
            "Bell Boy를 이용하시겠습니까?\n서비스 이용을 원하신다면, 짐을 두고 YES 버튼을 눌러주세요"
        )
        self.button_yes = QPushButton("YES")
        self.button_no = QPushButton("NO")

        self.button_yes.clicked.connect(self.yes_clicked)
        self.button_no.clicked.connect(self.no_clicked)

        layout.addWidget(self.label, alignment=Qt.AlignCenter)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_yes)
        button_layout.addWidget(self.button_no)

        layout.addLayout(button_layout)
        self.setLayout(layout)

    def yes_clicked(self):
        self.ros_node.publish_customer_decision(True)
        self.parent_widget.setCurrentIndex(2)  # 서비스 종료 화면으로 전환

    def no_clicked(self):
        self.ros_node.publish_customer_decision(False)
        self.parent_widget.setCurrentIndex(1)  # 서비스 이용 확인 화면 유지


class ServiceEndScreen(QWidget):
    def __init__(self, parent_widget, ros_node):
        super().__init__()
        self.parent_widget = parent_widget
        self.ros_node = ros_node
        self.setWindowTitle("Service End")

        layout = QVBoxLayout()

        self.label = QLabel(
            "목적지에 도착하셨다면, 짐을 들고 도착 버튼을 눌러주세요"
        )
        self.button_arrived = QPushButton("도착")
        self.button_arrived.clicked.connect(self.arrived_clicked)

        layout.addWidget(self.label, alignment=Qt.AlignCenter)
        layout.addWidget(self.button_arrived, alignment=Qt.AlignCenter)
        self.setLayout(layout)

    def arrived_clicked(self):
        self.ros_node.publish_service_terminate()
        self.parent_widget.setCurrentIndex(1)  # 서비스 이용 확인 화면으로 전환


class MainApp(QWidget):
    def __init__(self):
        super().__init__()

        # ROS2 초기화
        self.ros_node = ROSNode()

        self.setWindowTitle("Robot Interface")
        self.stacked_widget = QStackedWidget()

        # 화면 추가
        self.login_screen = LoginScreen(self.stacked_widget)
        self.service_confirm_screen = ServiceConfirmScreen(self.stacked_widget, self.ros_node)
        self.service_end_screen = ServiceEndScreen(self.stacked_widget, self.ros_node)

        self.stacked_widget.addWidget(self.login_screen)
        self.stacked_widget.addWidget(self.service_confirm_screen)
        self.stacked_widget.addWidget(self.service_end_screen)

        layout = QVBoxLayout()
        layout.addWidget(self.stacked_widget)
        self.setLayout(layout)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    main_app = MainApp()
    main_app.show()

    # ROS2 스핀 처리
    def ros_spin():
        rclpy.spin_once(main_app.ros_node, timeout_sec=0.1)

    timer = app.startTimer(100)
    app.timerEvent = lambda e: ros_spin()

    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
