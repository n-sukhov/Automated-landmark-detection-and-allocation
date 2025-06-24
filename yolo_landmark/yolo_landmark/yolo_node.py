import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

class YOLOLandmarkNode(Node):
    def __init__(self):
        super().__init__('yolo_landmark_node')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.annotated_pub = self.create_publisher(
            Image,
            '/yolo/annotated_image',
            10)
        self.get_logger().info('YOLO Landmark Node has been started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            results = self.model(cv_image, verbose=False)
            annotated_frame = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame)
            self.annotated_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOLandmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
