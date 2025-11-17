import rclpy
from rclpy.node import Node
from custom_message.msg import SkeletonList
from is_wire.core import Channel, Subscription
from is_msgs.image_pb2 import ObjectAnnotations
from .utils.is2ros import create_ros2_skeleton_list


class SkeletonPublisher(Node):
    """
    Nó ROS 2 que publica as mensagens SkeletonList (esqueletos no ROS) recebidas dos ObjectAnnotations (esqueletos no espaço).
    """

    def __init__(self):
        super().__init__("skeleton_list_publisher")

        self.channel = Channel("amqp://10.20.5.2:30000")
        self.subscription = Subscription(self.channel)
        self.subscription.subscribe(topic="SkeletonsGrouper.0.Localization")
        self.TIME_OUT = 0.1
        self.ROS_FRAME_ID = "Map"

        self.publisher_ = self.create_publisher(SkeletonList, "/skeleton_list", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Nó SkeletonPublisher iniciado e aguardando mensagens.")

    def timer_callback(self):
        """Função chamada pelo timer para publicar."""
        try:
            is_skeletons_packed = self.channel.consume(timeout=self.TIME_OUT)
            is_skeletons = is_skeletons_packed.unpack(ObjectAnnotations)

            if len(is_skeletons.objects) == 0:
                self.get_logger().debug("Mensagem recebida, mas não contém esqueletos.")
                return

            is_skeletons_list = is_skeletons.objects

            skeleton_list_msg = create_ros2_skeleton_list(
                is_skeletons=is_skeletons_list, frame_id=self.ROS_FRAME_ID
            )

            self.publisher_.publish(skeleton_list_msg)

            self.get_logger().info(
                f"Publicada mensagem SkeletonList com {len(skeleton_list_msg.skeletons)} esqueletos."
            )

        except TimeoutError:
            pass
        except Exception as e:
            self.get_logger().error(f"Erro durante o processamento (callback): {e}")


def main(args=None):
    rclpy.init(args=args)

    skeleton_publisher_node = SkeletonPublisher()

    try:
        rclpy.spin(skeleton_publisher_node)
    except KeyboardInterrupt:
        pass

    skeleton_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
