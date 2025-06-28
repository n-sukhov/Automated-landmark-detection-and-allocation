# save_markers.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import pickle
import os
from datetime import datetime

class MarkerSaver(Node):
    def __init__(self):
        super().__init__('marker_saver')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/yolo/objects_array',
            self.marker_callback,
            10)
        self.markers = None
        
    def marker_callback(self, msg):
        self.markers = msg
        self.get_logger().info('Received markers')
        
    def save_markers(self):
        if self.markers:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f'markers_{timestamp}.pkl'
            with open(filename, 'wb') as f:
                pickle.dump(self.markers, f)
            self.get_logger().info(f'Saved markers to {filename}')
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = MarkerSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.save_markers():
            node.get_logger().info('Markers saved successfully')
        else:
            node.get_logger().warn('No markers to save')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()