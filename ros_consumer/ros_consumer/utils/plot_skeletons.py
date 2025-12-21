import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import cv2
import colorsys

from custom_message.msg import SkeletonList

from is_msgs.image_pb2 import HumanKeypoints as HKP

matplotlib.use("Agg")


class ROSSkeletonVisualizer(Node):
    def __init__(self):
        super().__init__("ros_skeleton_visualizer")

        self.subscription = self.create_subscription(
            SkeletonList, "/skeleton_list", self.listener_callback, 10
        )

        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(projection="3d")

        # --- ESTES SÃO OS LINKS EXATOS DO SEU CÓDIGO ORIGINAL ---
        # Usamos HKP.Value para garantir que o ID numérico seja o correto
        self.links = [
            (HKP.Value("LEFT_SHOULDER"), HKP.Value("RIGHT_SHOULDER")),
            (HKP.Value("LEFT_SHOULDER"), HKP.Value("LEFT_HIP")),
            (HKP.Value("RIGHT_SHOULDER"), HKP.Value("RIGHT_HIP")),
            (HKP.Value("LEFT_HIP"), HKP.Value("RIGHT_HIP")),
            (HKP.Value("LEFT_SHOULDER"), HKP.Value("LEFT_EAR")),
            (HKP.Value("RIGHT_SHOULDER"), HKP.Value("RIGHT_EAR")),
            (HKP.Value("LEFT_SHOULDER"), HKP.Value("LEFT_ELBOW")),
            (HKP.Value("LEFT_ELBOW"), HKP.Value("LEFT_WRIST")),
            (HKP.Value("LEFT_HIP"), HKP.Value("LEFT_KNEE")),
            (HKP.Value("LEFT_KNEE"), HKP.Value("LEFT_ANKLE")),
            (HKP.Value("RIGHT_SHOULDER"), HKP.Value("RIGHT_ELBOW")),
            (HKP.Value("RIGHT_ELBOW"), HKP.Value("RIGHT_WRIST")),
            (HKP.Value("RIGHT_HIP"), HKP.Value("RIGHT_KNEE")),
            (HKP.Value("RIGHT_KNEE"), HKP.Value("RIGHT_ANKLE")),
            (HKP.Value("NOSE"), HKP.Value("LEFT_EYE")),
            (HKP.Value("LEFT_EYE"), HKP.Value("LEFT_EAR")),
            (HKP.Value("NOSE"), HKP.Value("RIGHT_EYE")),
            (HKP.Value("RIGHT_EYE"), HKP.Value("RIGHT_EAR")),
        ]

        self.get_logger().info(
            "Visualizador iniciado com os links originais do App (HKP)."
        )

    def _id_to_rgb_color(self, id):
        hue = (id % 20) / 20
        r, g, b = [x for x in colorsys.hls_to_rgb(hue, 0.6, 0.8)]
        return r, g, b

    def render_skeletons_3d(self, msg):
        for skeleton in msg.skeletons:
            parts = {
                j.id: (j.position.x, j.position.y, j.position.z)
                for j in skeleton.joints
            }
            color = self._id_to_rgb_color(id=skeleton.skeleton_id)

            if not parts:
                continue

            pts = np.array(list(parts.values()))
            self.ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=color, s=25)

            for begin, end in self.links:
                if begin in parts and end in parts:
                    x_p = [parts[begin][0], parts[end][0]]
                    y_p = [parts[begin][1], parts[end][1]]
                    z_p = [parts[begin][2], parts[end][2]]
                    self.ax.plot(x_p, y_p, zs=z_p, linewidth=3, color=color)

    def listener_callback(self, msg: SkeletonList):
        self.ax.clear()

        self.ax.view_init(azim=0, elev=20)
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-4, 4)
        self.ax.set_zlim(-0.25, 2)

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.render_skeletons_3d(msg)

        self.fig.canvas.draw()
        buf = np.asarray(self.fig.canvas.buffer_rgba())
        view_3d = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)

        cv2.imshow("Visualizador de Esqueletos ROS 2", view_3d)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ROSSkeletonVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
