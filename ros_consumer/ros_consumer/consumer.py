import rclpy
from rclpy.node import Node

# Importa a mensagem de nível mais alto que o publicador envia
from custom_message.msg import SkeletonList


class SkeletonListSubscriber(Node):

    def __init__(self):
        super().__init__("skeleton_list_echo_subscriber")

        self.subscription = self.create_subscription(
            SkeletonList,
            "/skeleton_list",
            self.listener_callback,
            10,
        )
        self.get_logger().info(
            "Nó Subscriber iniciado, ouvindo em '/skeleton_list'."
        )

    def listener_callback(self, msg: SkeletonList):
        """
        Callback chamada quando uma mensagem SkeletonList é recebida.
        Imprime o conteúdo da mensagem, simulando um 'topic echo'.
        """

        num_skeletons = len(msg.skeletons)
        self.get_logger().info(f"==================================================")
        self.get_logger().info(
            f"ROS 2 ECHO: Received SkeletonList with {num_skeletons} skeletons"
        )

        for i, skeleton in enumerate(msg.skeletons):

            skeleton_id = skeleton.skeleton_id
            num_joints = len(skeleton.joints)

            self.get_logger().info(
                f"  [Skeleton {i+1}/{num_skeletons} (ID: {skeleton_id})]: Total Joints: {num_joints}"
            )

            for joint in skeleton.joints:

                joint_id = joint.id
                x = joint.position.x
                y = joint.position.y
                z = joint.position.z

                self.get_logger().info(
                    f"    - Joint ID {joint_id}: Posição [X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f}]"
                )
        self.get_logger().info(f"==================================================\n")


def main(args=None):
    rclpy.init(args=args)
    skeleton_subscriber = SkeletonListSubscriber()

    try:
        rclpy.spin(skeleton_subscriber)
    except KeyboardInterrupt:
        pass

    skeleton_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
