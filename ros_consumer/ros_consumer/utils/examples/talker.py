import rclpy
from rclpy.node import Node

# Importa as mensagens personalizadas (Tipos singulares)
from custom_message.msg import SkeletonList, Skeleton, Joint

# Importa a mensagem padrão para a posição 3D
from geometry_msgs.msg import Point


class SkeletonListPublisher(Node):

    def __init__(self):
        super().__init__("skeleton_list_publisher")

        self.publisher_ = self.create_publisher(SkeletonList, "skeleton_data", 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        skeleton_list_msg = SkeletonList()

        skeletons = []

        # --- ESQUELETO 1 (Exemplo) ---
        skeleton1 = self._create_skeleton(
            skeleton_id=1,
            # Dados simulados: (id da junta, x, y, z)
            joint_data=[
                (0, 1.5, 0.5, 2.0),
                (1, 1.3, 0.4, 1.5),
                (2, 1.1, 0.3, 1.0),
            ],
            offset_y=0.0 + (self.frame_count * 0.01),  # Simulação de movimento
        )
        skeletons.append(skeleton1)

        # --- ESQUELETO 2 (Exemplo) ---
        skeleton2 = self._create_skeleton(
            skeleton_id=2,
            joint_data=[(0, -1.5, 0.5, 2.0), (1, -1.3, 0.4, 1.5), (2, -1.1, 0.3, 1.0)],
            offset_y=0.5 + (self.frame_count * 0.005),
        )
        skeletons.append(skeleton2)

        skeleton_list_msg.skeletons = skeletons

        self.publisher_.publish(skeleton_list_msg)

        self.get_logger().info(
            f"Publishing Frame {self.frame_count}. Total Skeletons: {len(skeleton_list_msg.skeletons)}."
        )
        self.frame_count += 1

    def _create_skeleton(self, skeleton_id, joint_data, offset_y):
        """Função auxiliar para criar uma mensagem Skeleton com Joints."""
        skeleton_msg = Skeleton()
        skeleton_msg.skeleton_id = skeleton_id

        joints_list = []
        for joint_id, x, y, z in joint_data:

            joint_msg = Joint()
            joint_msg.id = joint_id

            position_msg = Point()
            position_msg.x = float(x)
            position_msg.y = float(y) + offset_y
            position_msg.z = float(z)

            joint_msg.position = position_msg
            joints_list.append(joint_msg)

        skeleton_msg.joints = joints_list
        return skeleton_msg


def main(args=None):
    rclpy.init(args=args)
    skeleton_publisher = SkeletonListPublisher()
    rclpy.spin(skeleton_publisher)
    skeleton_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
