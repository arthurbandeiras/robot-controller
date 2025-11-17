import rclpy
from rclpy.node import Node

# Importa a mensagem de nível mais alto que o publicador envia
from custom_message.msg import SkeletonList


class SkeletonListSubscriber(Node):

    def __init__(self):
        super().__init__("skeleton_list_subscriber")

        # O subscriber deve usar o tipo SkeletonList
        self.subscription = self.create_subscription(
            SkeletonList, "skeleton_data", self.listener_callback, 10
        )
        self.subscription  # Previne o aviso de variável não utilizada

    def listener_callback(self, msg):
        """Callback chamada quando uma mensagem SkeletonList é recebida."""

        # 1. Acessa a lista de esqueletos (msg.skeletons é uma lista Python)
        num_skeletons = len(msg.skeletons)
        self.get_logger().info(
            f"--- Recebida mensagem com {num_skeletons} esqueletos ---"
        )

        # Loop 1: Itera sobre cada IS_Skeleton
        for skeleton in msg.skeletons:

            skeleton_id = skeleton.skeleton_id

            # 2. Acessa a lista de juntas (skeleton.joints é uma lista Python)
            num_joints = len(skeleton.joints)

            self.get_logger().info(
                f"  [Esqueleto ID: {skeleton_id}] Total de Juntas: {num_joints}"
            )

            # Loop 2: Itera sobre cada Joint
            for joint in skeleton.joints:

                # 3. Acessa os dados da junta e sua posição (sub-mensagem Point)
                joint_id = joint.id
                x = joint.position.x
                y = joint.position.y
                z = joint.position.z

                self.get_logger().info(
                    f"    - Junta ID {joint_id}: Posição (x={x:.3f}, y={y:.3f}, z={z:.3f})"
                )


def main(args=None):
    rclpy.init(args=args)
    skeleton_subscriber = SkeletonListSubscriber()
    rclpy.spin(skeleton_subscriber)
    skeleton_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
