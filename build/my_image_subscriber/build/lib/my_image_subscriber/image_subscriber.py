import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError  # Исправлено на CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/robotcar/radar/cart',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()  # Исправлено на CvBridge

    def listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            self.get_logger().info(f'subscribing data[0]: {data.data[0]}')  # Логирование
            cv2.imshow('Image', cv_image)  # Отображение изображения
            cv2.waitKey(1)  # Обновление окна
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')  # Логирование ошибки

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        image_subscriber.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Закрытие всех окон OpenCV

if __name__ == '__main__':
    main()