import rclpy
from rclpy.node import Node
from custom_message.msg import SkeletonList
import matplotlib.pyplot as plt
import numpy as np


class SkeletonPlotter(Node):
    def __init__(self):
        super().__init__("skeleton_plotter")

        self.subscription = self.create_subscription(
            SkeletonList, "/skeleton_list", self.listener_callback, 10
        )

        # Matplotlib init
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_zlim(0, 2.5)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.scatters = {}

    def listener_callback(self, msg: SkeletonList):
        self.ax.cla()
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_zlim(0, 2.5)

        for skeleton in msg.skeletons:
            xs, ys, zs = [], [], []
            for joint in skeleton.joints:
                xs.append(joint.position.x)
                ys.append(joint.position.y)
                zs.append(joint.position.z)

            self.ax.scatter(xs, ys, zs, s=20)

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
