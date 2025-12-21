import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import cv2
import colorsys
import time

from custom_message.msg import SkeletonList
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D 
from is_msgs.image_pb2 import HumanKeypoints as HKP

matplotlib.use("Agg")

class ROSSkeletonPotentialField(Node):
    def __init__(self):
        super().__init__("ros_skeleton_potential_field")

        self.a = 0.15
        self.b = 0.15
        self.n = 4
        self.kobs = 2.0
        self.range_map = 3.0 

        self.pose_robo = [0.0, 0.0, 0.0] 
        self.obstaculos = [] 
        
        self.sub_skeleton = self.create_subscription(SkeletonList, "/skeleton_list", self.skeleton_callback, 10)
        self.sub_aruco = self.create_subscription(Pose2D, "/ArucoPose2D", self.aruco_callback, 10)
        self.pub_potencial = self.create_publisher(Float32MultiArray, "/desvio_potencial", 10)

        #self.fig = plt.figure(figsize=(8, 8))
        self.fig = plt.figure(figsize=(19.2, 10.8), dpi=100)
        self.ax = self.fig.add_subplot(projection="3d")

        self.links = [
            (HKP.Value("LEFT_SHOULDER"), HKP.Value("RIGHT_SHOULDER")),
            (HKP.Value("LEFT_SHOULDER"), HKP.Value("LEFT_HIP")),
            (HKP.Value("RIGHT_SHOULDER"), HKP.Value("RIGHT_HIP")),
            (HKP.Value("LEFT_HIP"), HKP.Value("RIGHT_HIP")),
            (HKP.Value("LEFT_SHOULDER"), HKP.Value("LEFT_ELBOW")),
            (HKP.Value("LEFT_ELBOW"), HKP.Value("LEFT_WRIST")),
            (HKP.Value("LEFT_HIP"), HKP.Value("LEFT_KNEE")),
            (HKP.Value("LEFT_KNEE"), HKP.Value("LEFT_ANKLE")),
            (HKP.Value("RIGHT_SHOULDER"), HKP.Value("RIGHT_ELBOW")),
            (HKP.Value("RIGHT_ELBOW"), HKP.Value("RIGHT_WRIST")),
            (HKP.Value("RIGHT_HIP"), HKP.Value("RIGHT_KNEE")),
            (HKP.Value("RIGHT_KNEE"), HKP.Value("RIGHT_ANKLE")),
            (HKP.Value("NOSE"), HKP.Value("LEFT_EYE")),
            (HKP.Value("NOSE"), HKP.Value("RIGHT_EYE"))
        ]

        self.get_logger().info("Visualizador iniciado. Robô agora é uma flecha.")

    def aruco_callback(self, msg):
        self.pose_robo = [msg.x, msg.y, msg.theta]

    def _id_to_rgb_color(self, id):
        hue = (id % 20) / 20
        return colorsys.hls_to_rgb(hue, 0.6, 0.8)

    def calcular_v_repulsiva(self, robot_x, robot_y, obs_x, obs_y):
        dx = robot_x - obs_x
        dy = robot_y - obs_y
        H = np.exp(-(dx**self.n)/self.a) * np.exp(-(dy**self.n)/self.b)
        v_grad_x = -H * self.n * (dx**(self.n - 1)) / self.a
        v_grad_y = -H * self.n * (dy**(self.n - 1)) / self.b
        Jobs = np.array([[v_grad_x, v_grad_y]])
        denom = np.dot(Jobs, Jobs.T) + 0.1
        Jobs_pinv = Jobs.T / denom
        xref_obs = -Jobs_pinv * self.kobs * H
        return xref_obs.flatten() 

    def skeleton_callback(self, msg: SkeletonList):
        self.ax.clear()
        self.obstaculos = []
        v_total = np.array([0.0, 0.0])

        ID_KNEE_L = HKP.Value("LEFT_KNEE")
        ID_KNEE_R = HKP.Value("RIGHT_KNEE")

        for skeleton in msg.skeletons:
            parts = {j.id: (j.position.x, j.position.y, j.position.z) for j in skeleton.joints}
            color = self._id_to_rgb_color(skeleton.skeleton_id)
            if not parts: continue

            for k_id in [ID_KNEE_L, ID_KNEE_R]:
                if k_id in parts:
                    ox, oy, _ = parts[k_id]
                    self.obstaculos.append((ox, oy))
                    v_rep = self.calcular_v_repulsiva(self.pose_robo[0], self.pose_robo[1], ox, oy)
                    v_total += v_rep

            pts = np.array(list(parts.values()))
            self.ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=color, s=20)
            for b, e in self.links:
                if b in parts and e in parts:
                    self.ax.plot([parts[b][0], parts[e][0]], [parts[b][1], parts[e][1]], [parts[b][2], parts[e][2]], color=color, linewidth=2)

        self.render_heatmap()

        res_msg = Float32MultiArray()
        res_msg.data = [float(v_total[0]), float(v_total[1])]
        self.pub_potencial.publish(res_msg)

        self.finalize_plot()

    def render_heatmap(self):
        grid_res = 30
        x = np.linspace(-self.range_map, self.range_map, grid_res)
        y = np.linspace(-self.range_map, self.range_map, grid_res)
        X, Y = np.meshgrid(x, y)
        Z_pot = np.zeros_like(X)

        for (ox, oy) in self.obstaculos:
            DX = X - ox
            DY = Y - oy
            H = np.exp(-(DX**self.n)/self.a) * np.exp(-(DY**self.n)/self.b)
            Z_pot += H

        if self.obstaculos:
            self.ax.contourf(X, Y, Z_pot, zdir='z', offset=-0.05, cmap='Reds', alpha=0.5)

        # --- NOVA REPRESENTAÇÃO DO ROBÔ (FLECHA) ---
        rx, ry, rt = self.pose_robo
        u = np.cos(rt) # Componente X do vetor unitário
        v = np.sin(rt) # Componente Y do vetor unitário
        print(f"Robô em x: {rx:.2f}, y: {ry:.2f}, theta: {rt:.2f} radianos")

        # ax.quiver(x, y, z, u, v, w)
        self.ax.quiver(rx, ry, 0, u, v, 0, 
                       length=0.5, normalize=True, color='blue', 
                       linewidth=3, label='Robô', pivot='tail')

    def finalize_plot(self):
        self.ax.set_xlim(-self.range_map, self.range_map)
        self.ax.set_ylim(-self.range_map, self.range_map)
        self.ax.set_zlim(-0.1, 2.0)
        self.ax.view_init(azim=0, elev=20)
        
        self.fig.canvas.draw()
        img = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        cv2.imshow("Heatmap de Campo Potencial", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ROSSkeletonPotentialField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()