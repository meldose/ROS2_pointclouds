import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# Simulated ECS structure
class ECSWorld:
    def __init__(self):
        self.entities = {}

    def create_entity(self, name, component):
        self.entities[name] = component

    def get_entities(self):
        return self.entities

# ROS2 Subscriber Node
class PointCloudSubscriber(Node):
    def __init__(self, ecs_world):
        super().__init__('pointcloud_subscriber')
        self.ecs_world = ecs_world
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud_topic',
            self.pointcloud_callback,
            10)
        self.get_logger().info("PointCloud2 subscriber initialized.")

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to list of (x, y, z)
        point_list = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if point_list.size > 0:
            entity_name = f"PointCloud_{msg.header.stamp.sec}"
            self.ecs_world.create_entity(entity_name, point_list)

            self.get_logger().info(f"Received {len(point_list)} points, stored in ECS.")

# Main function
def main():
    rclpy.init()
    ecs_world = ECSWorld()
    node = PointCloudSubscriber(ecs_world)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    # Print stored ECS entities (debugging)
    print("\nECS Entities:")
    for name, points in ecs_world.get_entities().items():
        print(f"{name}: {len(points)} points")

if __name__ == '__main__':
    main()