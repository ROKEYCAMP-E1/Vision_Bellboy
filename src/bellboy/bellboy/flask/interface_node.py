import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import requests

class InterfaceNode(Node):
    def __init__(self):
        super().__init__('interface_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/amr_video',
            self.listener_callback,
            10
        )
        self.http_url = "http://127.0.0.1:5000/amr_cam"  # manager_interface.py의 엔드포인트 URL
        self.get_logger().info("Interface Node Initialized. Subscribed to /amr_video")

    def listener_callback(self, msg):
        """Callback function to process incoming messages."""
        try:
            # ROS2 메시지 데이터를 HTTP POST 요청으로 전송
            response = requests.post(
                self.http_url,
                files={"frame": msg.data}
            )
            if response.status_code == 200:
                self.get_logger().info("Frame sent to manager_interface.")
            else:
                self.get_logger().error(f"Failed to send frame. Status code: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error while sending frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    interface_node = InterfaceNode()
    try:
        rclpy.spin(interface_node)
    except KeyboardInterrupt:
        interface_node.get_logger().info("Shutting down Interface Node.")
    finally:
        interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
