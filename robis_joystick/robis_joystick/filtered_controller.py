import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseArray
from turtlesim.msg import Pose as TurtlePose
from std_msgs.msg import Float32MultiArray
import numpy as np


class RobisLookAheadFollower(Node):
    def __init__(self):
        super().__init__("robis_look_ahead_follower")

        # --- Publishers e Subscribers ---
        self.publisher_cmd = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscription_turtle = self.create_subscription(
            TurtlePose, "/turtle1/pose", self.turtle_pose_callback, 10
        )
        self.subscription_path = self.create_subscription(
            PoseArray, "/trajetoria_alvo", self.path_callback, 10
        )
        self.subscription_potencial = self.create_subscription(
            Float32MultiArray, "/desvio_potencial", self.potencial_callback, 10
        )
        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )

        # --- Estado ---
        self.pose_robo = None
        self.trajetoria = []
        self.indice_alvo = 0
        self.vel_pot_global = np.array([0.0, 0.0])

        # --- Modo de Controle (Manual/Autônomo) ---
        self.pass_through_mode = False
        self.last_joy_msg = None
        self.prev_button_y = False

        # --- Parâmetros Look-ahead + Controle---
        self.Ld = 0.1  # Look-ahead distance
        self.v_max = 1.5  # Velocidade linear máxima
        self.kp_angular = 5.0  # Ganho proporcional de giro
        self.kd_angular = 0.4  # Ganho derivativo de giro
        self.erro_anterior = 0.0
        self.dt = 0.02  # Intervalo de tempo do timer
        self.k_evasao = 1.5  # Ganho para o desvio lateral do campo potencial

        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Seguidor Look-ahead iniciado.")

    def turtle_pose_callback(self, msg):
        self.pose_robo = [msg.x, msg.y, msg.theta]

    def path_callback(self, msg):
        self.trajetoria = [
            [
                p.position.x,
                p.position.y,
                2.0 * np.arctan2(p.orientation.z, p.orientation.w),
            ]
            for p in msg.poses
        ]
        self.indice_alvo = 0
        self.get_logger().info(
            f"Trajetória com {len(self.trajetoria)} pontos carregada."
        )

    def joy_callback(self, msg):
        """Lógica de callback para o joystick, incluindo toggle do modo de controle
        com o botão Y."""
        if msg.buttons[3] and not self.prev_button_y:
            self.pass_through_mode = not self.pass_through_mode
            status = "AUTÔNOMO" if self.pass_through_mode else "MANUAL"
            self.get_logger().info(f">>> MODO {status} ATIVADO")
        self.prev_button_y = bool(msg.buttons[3])
        self.last_joy_msg = msg

    def potencial_callback(self, msg):
        """Recebe o vetor de velocidade de desvio potencial do campo de obstáculos
        como [vx, vy] e o armazena para uso no controle autônomo."""
        if len(msg.data) >= 2:
            self.vel_pot_global = np.array([msg.data[0], msg.data[1]])

    def atualizar_look_ahead(self):
        """Busca o ponto na trajetória que está mais próximo da distância Ld"""
        if (not self.trajetoria) or (self.pose_robo is None):
            return None

        rx, ry, _ = self.pose_robo

        # Percorre a trajetória a partir do índice atual para encontrar o melhor ponto
        for i in range(self.indice_alvo, len(self.trajetoria)):
            tx, ty, _ = self.trajetoria[i]
            dist = np.hypot(tx - rx, ty - ry)
            # Se o ponto está além da distância Ld, ele vira nosso alvo visual
            if dist > self.Ld:
                self.indice_alvo = i
                return self.trajetoria[i]

        # Se não houver pontos além de Ld, retorna o último ponto da trajetória
        return self.trajetoria[-1]

    def calcular_comando_autonomo(self):
        if self.pose_robo is None or not self.trajetoria:
            return 0.0, 0.0

        alvo = self.atualizar_look_ahead()

        if alvo is None:
            return 0.0, 0.0

        tx, ty, t_theta = alvo
        rx, ry, rt = self.pose_robo

        # Vetor para o alvo
        dx, dy = tx - rx, ty - ry
        dist_total = np.hypot(dx, dy)

        # Ângulo para o alvo
        theta_alvo = np.arctan2(dy, dx)
        erro_theta = np.arctan2(np.sin(theta_alvo - rt), np.cos(theta_alvo - rt))

        # Termo derivativo
        d_erro = (erro_theta - self.erro_anterior) / self.dt
        self.erro_anterior = erro_theta

        # Para caso esteja próximo do fim da trajetória
        if self.indice_alvo >= len(self.trajetoria) - 1 and dist_total < 0.2:
            erro_final = np.arctan2(np.sin(t_theta - rt), np.cos(t_theta - rt))
            return 0.0, self.kp_angular * erro_final + self.kd_angular * d_erro

        # --- Lógica Look-ahead ---
        v_final_traj = self.v_max * np.cos(erro_theta) ** 3

        # Se estiver muito perto de um waypoint intermediário, não desacelerar bruscamente
        w_final_traj = self.kp_angular * erro_theta + self.kd_angular * d_erro

        # Rotação do Campo Potencial: Global -> Local do Robô
        vx_global, vy_global = self.vel_pot_global
        vx_local = vx_global * np.cos(rt) + vy_global * np.sin(rt)
        vy_local = -vx_global * np.sin(rt) + vy_global * np.cos(rt)

        # Fusão do comando de trajetória com o desvio potencial
        v_final = v_final_traj + vx_local
        w_final = w_final_traj + (vy_local * self.k_evasao)

        return v_final, w_final

    def control_loop(self):
        cmd = Twist()

        if not self.pass_through_mode:
            if self.last_joy_msg is None:
                return
            cmd.linear.x = self.last_joy_msg.axes[1] * self.v_max
            cmd.angular.z = self.last_joy_msg.axes[3] * self.kp_angular
            self.publisher_cmd.publish(cmd)
            return

        v, w = self.calcular_comando_autonomo()
        cmd.linear.x = float(np.clip(v, -1.0, 1.0))
        cmd.angular.z = float(np.clip(w, -3.0, 3.0))
        self.publisher_cmd.publish(cmd)


def main():
    rclpy.init()
    node = RobisLookAheadFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
