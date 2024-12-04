import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class AmrSubscriber(Node):
    def __init__(self):
        super().__init__('AmrSubscriber')
        self.subscription= self.create_subscription(Image,'amr_image',self.image_callback,10)
        self.subscription
        self.bridge=CvBridge()

    def image_callback(self,msg):
        self.get_logger().info('received')
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Received Image', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()  # Exit if 'q' is pressed

def main(args=None):
    rclpy.init(args=args)
    amr_subscriber = AmrSubscriber()

    try:
        rclpy.spin(amr_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  # Close OpenCV windows
        rclpy.shutdown()

if __name__ == '__main__':
    main()
