import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header


class CirclePublisher(Node):
    def __init__(self):
        super().__init__("circle_publisher")

        self.publisher_ = self.create_publisher(PoseArray, "/trajetoria_alvo", 10)

        self.radius = 3.0  # Raio do círculo
        self.num_pontos = 100  # Quantidade de waypoints
        self.center_x = 5.54  # Centro do círculo (ajuste conforme necessário)
        self.center_y = 5.54

        self.timer = self.create_timer(5.0, self.publish_path)
        self.get_logger().info("Publicador PoseArray (Círculo) iniciado.")

    def publish_path(self):
        msg = PoseArray()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (
            "world"  # Certifique-se de que este frame existe no seu TF
        )

        t = np.linspace(0, 2 * np.pi, self.num_pontos)
        x = self.center_x + self.radius * np.cos(t)
        y = self.center_y + self.radius * np.sin(t)

        for i in range(self.num_pontos):
            pose = Pose()

            pose.position.x = float(x[i])
            pose.position.y = float(y[i])
            pose.position.z = 0.0

            # Orientação (Theta calculado pela tangente do próximo ponto)
            if i < self.num_pontos - 1:
                theta = np.arctan2(y[i + 1] - y[i], x[i + 1] - x[i])
            else:
                # No último ponto, mantém a direção do anterior
                theta = np.arctan2(y[i] - y[i - 1], x[i] - x[i - 1])

            # Conversão Euler (theta) -> Quatérnio (Z e W para robô 2D)
            pose.orientation.z = float(np.sin(theta / 2.0))
            pose.orientation.w = float(np.cos(theta / 2.0))
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0

            msg.poses.append(pose)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publicada trajetória com {len(msg.poses)} Poses.")


def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
