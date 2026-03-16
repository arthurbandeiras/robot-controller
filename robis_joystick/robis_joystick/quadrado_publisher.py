import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header


class QuadradoPublisher(Node):
    def __init__(self):
        super().__init__("quadrado_publisher")

        self.publisher_ = self.create_publisher(PoseArray, "/trajetoria_alvo", 10)

        # Parâmetros do Quadrado
        self.lado = 4.0  # Comprimento do lado do quadrado
        self.pontos_por_lado = 25

        # Centralização para Turtlesim (5.54 é o centro)
        self.offset_x = 5.54 - (self.lado / 2.0)
        self.offset_y = 5.54 - (self.lado / 2.0)

        self.timer = self.create_timer(5.0, self.publish_path)
        self.get_logger().info("Publicador de Trajetória Quadrada iniciado.")

    def publish_path(self):
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Definição dos vértices (relativos ao offset)
        # Ordem: (0,0) -> (L,0) -> (L,L) -> (0,L) -> volta ao (0,0)
        vertices = [
            (0.0, 0.0),
            (self.lado, 0.0),
            (self.lado, self.lado),
            (0.0, self.lado),
            (0.0, 0.0),
        ]

        lista_pontos = []

        # Gera pontos intermediários entre os vértices para o Look-ahead não "pular" o lado
        for i in range(len(vertices) - 1):
            p1 = vertices[i]
            p2 = vertices[i + 1]

            x_vals = np.linspace(p1[0], p2[0], self.pontos_por_lado, endpoint=False)
            y_vals = np.linspace(p1[1], p2[1], self.pontos_por_lado, endpoint=False)

            for j in range(len(x_vals)):
                lista_pontos.append(
                    (x_vals[j] + self.offset_x, y_vals[j] + self.offset_y)
                )

        # Criar as mensagens Pose
        for i in range(len(lista_pontos)):
            pose = Pose()
            px, py = lista_pontos[i]

            pose.position.x = float(px)
            pose.position.y = float(py)

            # Cálculo da orientação (theta)
            if i < len(lista_pontos) - 1:
                p_next = lista_pontos[i + 1]
                theta = np.arctan2(p_next[1] - py, p_next[0] - px)
            else:
                theta = 0.0  # Último ponto

            pose.orientation.z = float(np.sin(theta / 2.0))
            pose.orientation.w = float(np.cos(theta / 2.0))

            msg.poses.append(pose)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Trajetória quadrada enviada com {len(msg.poses)} pontos."
        )


def main(args=None):
    rclpy.init(args=args)
    node = QuadradoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
