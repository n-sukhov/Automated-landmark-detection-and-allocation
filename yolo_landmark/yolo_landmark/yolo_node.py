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
    if abs(array[idx] - value) > 0.2:
        return None
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
        self.marker_array = MarkerArray()
        self.current_marker_id = 0 
        self.object_positions = {}
        self.get_logger().info('YOLO Landmark Node has been started')

    def scan_callback(self, msg):
        self.last_scan = msg

    def is_near_existing_object(self, class_name, x, y, threshold=0.1):
        if class_name not in self.object_positions:
            return False
            
        for pos in self.object_positions[class_name]:
            dx = pos['x'] - x
            dy = pos['y'] - y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < threshold:
                return True
        return False

    def update_object_positions(self, class_name, x, y):
        if class_name not in self.object_positions:
            self.object_positions[class_name] = []
        
        self.object_positions[class_name].append({'x': x, 'y': y})

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
        
            new_markers = MarkerArray()

            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0].item())
                    class_name = self.model.names[class_id]
                    confidence = box.conf[0].item()

                    x_center = int((box.xyxy[0][0].item() + box.xyxy[0][2].item()) / 2)
                    angle = (x_center / img_width - 0.5) * fov_rad
                    scan = self.last_scan
                    scan_angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
                    index = find_nearest(scan_angles, angle)
                    if index is not None:
                        distance = scan.ranges[index]
                        if not math.isfinite(distance):
                            continue

                        x = distance * math.cos(angle)
                        y = - distance * math.sin(angle)
                        z = 0.0

                        if self.is_near_existing_object(class_name, x, y):
                            continue

                        self.update_object_positions(class_name, x, y)

                        marker = Marker()
                        marker.header.frame_id = "odom"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.id = self.current_marker_id
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

                        new_markers.markers.append(marker)
                        self.current_marker_id += 1

                        text_marker = Marker()
                        text_marker.header.frame_id = "odom"
                        text_marker.header.stamp = self.get_clock().now().to_msg()
                        text_marker.id = self.current_marker_id
                        text_marker.type = Marker.TEXT_VIEW_FACING
                        text_marker.action = Marker.ADD
                        text_marker.pose.position.x = x
                        text_marker.pose.position.y = y
                        text_marker.pose.position.z = z + 0.3
                        text_marker.scale.z = 0.3
                        text_marker.color.a = 1.0
                        text_marker.color.r = 1.0
                        text_marker.color.g = 1.0
                        text_marker.color.b = 1.0
                        text_marker.text = f"{class_name}({confidence:.2f})"

                        new_markers.markers.append(text_marker)
                        self.current_marker_id += 1

                        self.get_logger().info(
                            f"Published object point at angle {round(math.degrees(angle), 1)}°: ({round(x, 2)}, {round(y, 2)})"
                        )

            self.marker_array.markers.extend(new_markers.markers)
            self.marker_pub.publish(self.marker_array)

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
