import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class RobisFinalControl(Node):
    def __init__(self):
        super().__init__('robis_final_control')

        # --- Publishers e Subscribers Atualizados ---
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscription_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.subscription_aruco = self.create_subscription(Pose2D, '/ArucoPose2D', self.aruco_callback, 10)
        self.subscription_potencial = self.create_subscription(Float32MultiArray, '/desvio_potencial', self.potencial_callback, 10)

        # --- Estado do Sistema ---
        self.pose_robo = None           # [x, y, theta] vindo da Pose2D
        self.vel_pot_global = np.array([0.0, 0.0])
        self.pass_through_mode = False  # Toggle com botão Y
        self.prev_button_y = False
        self.last_joy_msg = None
        
        # --- Parâmetros de Trajetória ---
        self.a = 1.5                    # Tamanho da lemniscata
        self.periodo = 40.0             # Segundos para o "8"
        self.w_freq = (2 * np.pi) / self.periodo
        self.t_inicial_autonomo = None
        
        # --- Ganhos de Controle ---
        self.kp_linear = 0.5
        self.kp_angular = 2.0
        self.k_evasao = 1.5             # Ganho para o desvio lateral
        self.k_speed_manual = 0.3       # Velocidade máxima no joystick

        # Timer de controle a 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Controlador Robis Final Iniciado.")
        self.get_logger().info("Use o botão Y para alternar entre Manual e Lemniscata + Obstáculos.")

    def aruco_callback(self, msg):
        # Captura exata do Pose2D conforme o ros2 topic echo
        self.pose_robo = [msg.x, msg.y, msg.theta]

    def joy_callback(self, msg):
        self.last_joy_msg = msg
        # Lógica de Toggle (Botão Y)
        if msg.buttons[3] and not self.prev_button_y:
            self.pass_through_mode = not self.pass_through_mode
            if self.pass_through_mode:
                self.t_inicial_autonomo = time.time()
                self.get_logger().info(">>> MODO AUTÔNOMO ATIVADO (Lemniscata + Campo Potencial)")
            else:
                self.get_logger().info(">>> MODO MANUAL ATIVADO")
        self.prev_button_y = bool(msg.buttons[3])

    def potencial_callback(self, msg):
        # Recebe [vx, vy] do heatmap/esqueletos
        if len(msg.data) >= 2:
            self.vel_pot_global = np.array([msg.data[0], msg.data[1]])

    def calcular_comando_autonomo(self):
        if self.pose_robo is None:
            return 0.0, 0.0

        # 1. Referência da Lemniscata (Bernoulli)
        t = time.time() - self.t_inicial_autonomo
        den = 1 + (np.sin(self.w_freq * t) ** 2)
        ref_x = (self.a * np.cos(self.w_freq * t)) / den
        ref_y = (self.a * np.sin(self.w_freq * t) * np.cos(self.w_freq * t)) / den

        # 2. Erro de Posição
        rx, ry, rt = self.pose_robo
        dx = ref_x - rx
        dy = ref_y - ry
        distancia = np.sqrt(dx**2 + dy**2)
        
        theta_alvo = np.arctan2(dy, dx)
        erro_theta = np.arctan2(np.sin(theta_alvo - rt), np.cos(theta_alvo - rt))

        # Comando base da trajetória
        v_traj = self.kp_linear * distancia * np.cos(erro_theta)
        w_traj = self.kp_angular * erro_theta

        # 3. Rotação do Campo Potencial (Global -> Local do Robô)
        # vx_l = vx_g*cos(th) + vy_g*sin(th)
        # vy_l = -vx_g*sin(th) + vy_g*cos(th)
        vx_g, vy_g = self.vel_pot_global
        vx_l = vx_g * np.cos(rt) + vy_g * np.sin(rt)
        vy_l = -vx_g * np.sin(rt) + vy_g * np.cos(rt)

        # 4. Fusão: Linear com avanço, Angular com desvio lateral
        v_final = v_traj + vx_l
        w_final = w_traj + (self.k_evasao * vy_l)

        return v_final, w_final

    def control_loop(self):
        cmd = Twist()

        if self.pass_through_mode:
            v, w = self.calcular_comando_autonomo()
            cmd.linear.x = float(np.clip(v, -1.0, 1.0))
            cmd.angular.z = float(np.clip(w, -2.0, 2.0))
        
        elif self.last_joy_msg:
            # Comando Manual (Eixo 1: Linear, Eixo 3: Angular)
            cmd.linear.x = self.last_joy_msg.axes[1] * self.k_speed_manual
            cmd.angular.z = self.last_joy_msg.axes[3] * self.k_speed_manual
        
        self.publisher_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobisFinalControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()