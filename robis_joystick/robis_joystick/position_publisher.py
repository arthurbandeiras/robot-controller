import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header


class PositionPublisher(Node):
    def __init__(self):
        super().__init__("position_publisher")

        self.publisher_ = self.create_publisher(PoseArray, "/trajetoria_alvo", 10)

        self.num_pontos = 1  # Quantidade de waypoints

        self.timer = self.create_timer(5.0, self.publish_path)
        self.get_logger().info("Publicador PoseArray (Círculo) iniciado.")

    def publish_path(self):
        msg = PoseArray()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        pose = Pose()
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.position.x = 1.0
        pose.position.y = 1.0
        pose.position.z = 0.0

        msg.poses.append(pose)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publicada trajetória com {len(msg.poses)} Poses.")


def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
