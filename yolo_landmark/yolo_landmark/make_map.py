#!/usr/bin/env python3
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node

class MapUpdaterNode(Node):
    def __init__(self):
        super().__init__('map_updater_node')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_modified', 10)
        self.last_map = None

        self.point_sub = self.create_subscription(PointStamped, '/yolo/object_position', self.point_callback, 10)

    def map_callback(self, msg):
        self.last_map = msg
            
    def point_callback(self, point_msg):
        if self.last_map is None:
            self.get_logger().warn("No map received yet.")
            return

        map_copy = OccupancyGrid()
        map_copy.header = self.last_map.header
        map_copy.info = self.last_map.info
        map_copy.data = list(self.last_map.data)  # копируем список, т.к. data - tuple

        x_map = point_msg.point.x
        y_map = point_msg.point.y
        origin_x = map_copy.info.origin.position.x
        origin_y = map_copy.info.origin.position.y
        resolution = map_copy.info.resolution
        width = map_copy.info.width
        height = map_copy.info.height

        i = int((x_map - origin_x) / resolution)
        j = int((y_map - origin_y) / resolution)

        if 0 <= i < width and 0 <= j < height:
            index = j * width + i
            map_copy.data[index] = 100  # отмечаем препятствие

            map_copy.header.frame_id = 'map'  # обязательно!
            now = self.get_clock().now()
            if now.nanoseconds == 0:
                self.get_logger().warn("Waiting for /clock time...")
                return

            # Используй время из последнего полученного сообщения карты или время из clock, если оно есть
            if self.last_map.header.stamp.sec != 0 or self.last_map.header.stamp.nanosec != 0:
                map_copy.header.stamp = self.last_map.header.stamp
            else:
                map_copy.header.stamp = now.to_msg()


            self.map_pub.publish(map_copy)
            self.get_logger().info(f"Updated map at cell ({i},{j})")
        else:
            self.get_logger().warn(f"Point ({x_map},{y_map}) out of map bounds")


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdaterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()