import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose2D, PoseArray
from turtlesim.msg import Pose as TurtlePose  # Para o Turtlesim
from std_msgs.msg import Float32MultiArray
import numpy as np


class RobisPathFollower(Node):
    def __init__(self):
        super().__init__("robis_path_follower")

        # --- Publishers e Subscribers ---
        # self.publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.publisher_cmd = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )
        # self.subscription_aruco = self.create_subscription(
        #    Pose2D, "/ArucoPose2D", self.aruco_callback, 10
        # )
        self.subscription_aruco = self.create_subscription(
            TurtlePose, "/turtle1/pose", self.turtle_pose_callback, 10
        )

        self.subscription_potencial = self.create_subscription(
            Float32MultiArray, "/desvio_potencial", self.potencial_callback, 10
        )

        self.subscription_path = self.create_subscription(
            PoseArray, "/trajetoria_alvo", self.path_callback, 10
        )

        # --- Estado do Sistema ---
        self.pose_robo = None
        self.vel_pot_global = np.array([0.0, 0.0])
        self.pass_through_mode = False
        self.prev_button_y = False
        self.last_joy_msg = None

        # --- Lógica de Waypoints ---
        self.trajetoria = []  # Lista de [x, y, theta]
        self.indice_alvo = 0  # Ponto atual da lista que estamos perseguindo
        self.dist_tolerancia = 0.05  # Distância (m) para considerar que chegou no ponto

        # --- Ganhos de Controle ---
        self.kp_linear = 1.5  # ganho robo: 0.6
        self.kp_angular = 5.0  # ganho robo: 2.5
        self.k_evasao = 1.2
        self.k_speed_manual = 0.4

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Seguidor de Trajetória Robis Iniciado.")

    def path_callback(self, msg):
        """
        Converte o PoseArray (Quatérnios) para a lista [[x, y, theta], ...]
        """
        nova_trajetoria = []

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.orientation.z
            w = pose.orientation.w
            theta = 2.0 * np.arctan2(z, w)

            nova_trajetoria.append([x, y, theta])

        self.trajetoria = nova_trajetoria
        self.indice_alvo = 0
        self.get_logger().info(
            f"Recebidos {len(self.trajetoria)} pontos via PoseArray."
        )

    def aruco_callback(self, msg):
        self.pose_robo = [msg.x, msg.y, msg.theta]

    def turtle_pose_callback(self, msg):
        self.pose_robo = [msg.x, msg.y, msg.theta]

    def joy_callback(self, msg):
        if msg.buttons[3] and not self.prev_button_y:
            self.pass_through_mode = not self.pass_through_mode
            status = "AUTÔNOMO" if self.pass_through_mode else "MANUAL"
            self.get_logger().info(f">>> MODO {status} ATIVADO")
        self.prev_button_y = bool(msg.buttons[3])
        self.last_joy_msg = msg

    def potencial_callback(self, msg):
        try:
            if len(msg.data) >= 2:
                self.vel_pot_global = np.array([msg.data[0], msg.data[1]])
        except Exception:
            pass

    def calcular_comando_autonomo(self):
        if self.pose_robo is None or not self.trajetoria:
            return 0.0, 0.0

        if self.indice_alvo >= len(self.trajetoria):
            # verificação de fim de trajetória
            _, _, ref_theta_final = self.trajetoria[-1]
            rt = self.pose_robo[2]
            erro_final_theta = np.arctan2(
                np.sin(ref_theta_final - rt), np.cos(ref_theta_final - rt)
            )

            if abs(erro_final_theta) > 0.1:
                return 0.0, self.kp_angular * erro_final_theta
            self.get_logger().info("Trajetória completa! Mantendo posição.")
            return 0.0, 0.0

        # Obter alvo atual
        ref_x, ref_y, _ = self.trajetoria[self.indice_alvo]
        rx, ry, rt = self.pose_robo

        # Cálculo de Distância
        dx = ref_x - rx
        dy = ref_y - ry
        distancia = np.sqrt(dx**2 + dy**2)

        # Lógica de troca de Waypoint (Sem interromper o fluxo)
        if distancia < self.dist_tolerancia:
            self.get_logger().info(f"Ponto {self.indice_alvo} alcançado!")
            self.indice_alvo += 1
            # Recursão simples para já calcular o próximo ponto no mesmo ciclo
            return self.calcular_comando_autonomo()

        # Ângulo para o alvo
        theta_alvo = np.arctan2(dy, dx)
        erro_theta = np.arctan2(np.sin(theta_alvo - rt), np.cos(theta_alvo - rt))

        # Controle Base (Lei de controle linear e angular)
        # Adicionado np.cos(erro_theta) para o robô diminuir a velocidade linear se estiver muito desalinhado
        v_base = self.kp_linear * distancia * np.cos(erro_theta)

        # Se o erro for maior que 90 graus, v_base ficaria negativo (o robô anda de ré).
        # Se não quiser que ele ande de ré, use: v_base = max(0.0, v_base)
        # v_base = max(0.0, v_base)

        w_base = self.kp_angular * erro_theta

        # Integração com Campo Potencial (Obstáculos)
        vx_g, vy_g = self.vel_pot_global
        vy_l = -vx_g * np.sin(rt) + vy_g * np.cos(rt)

        v_final = v_base
        w_final = w_base + (self.k_evasao * vy_l)

        return v_final, w_final

    def control_loop(self):
        cmd = Twist()
        if self.pass_through_mode:
            v, w = self.calcular_comando_autonomo()
            cmd.linear.x = float(np.clip(v, -0.6, 0.6))
            cmd.angular.z = float(np.clip(w, -1.5, 1.5))
        elif self.last_joy_msg:
            cmd.linear.x = self.last_joy_msg.axes[1] * self.k_speed_manual
            cmd.angular.z = self.last_joy_msg.axes[3] * self.k_speed_manual

        self.publisher_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobisPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
