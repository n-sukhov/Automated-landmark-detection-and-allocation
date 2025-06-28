import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

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
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/yolo/objects_array',
            10)
        
        self.last_scan = None
        self.get_logger().info('YOLO Landmark Node has been started')

    def scan_callback(self, msg):
        self.last_scan = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            results = self.model(cv_image, verbose=False)
            
            annotated_frame = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame)
            self.annotated_pub.publish(annotated_msg)

            if self.last_scan is None:
                self.get_logger().warn('No scan data yet.')
                return
            
            fov_rad = 1.047
            img_height, img_width = cv_image.shape[:2]
        
            marker_array = MarkerArray()
            marker_id = 0

            for result in results:
                for box in result.boxes:
                    x_center = int((box.xyxy[0][0].item() + box.xyxy[0][2].item()) / 2)
                    angle = (x_center / img_width - 0.5) * fov_rad
                    scan = self.last_scan
                    scan_angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
                    index = find_nearest(scan_angles, angle)

                    distance = scan.ranges[index]
                    if not math.isfinite(distance):
                        continue

                    x = distance * math.cos(angle)
                    y = distance * math.sin(angle)
                    z = 0.0

                    marker = Marker()
                    marker.header.frame_id = "odom"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.id = marker_id
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = z
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    
                    marker_array.markers.append(marker)
                    marker_id += 1

                    self.get_logger().info(
                        f"Published object point at angle {round(math.degrees(angle), 1)}°: ({round(x, 2)}, {round(y, 2)})"
                    )   
            
            if len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)

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
