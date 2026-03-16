import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose
from std_msgs.msg import Float32MultiArray
import numpy as np


class ObstacleTurtle(Node):
    def __init__(self):
        super().__init__("robis_look_ahead_follower")

        # --- Publishers e Subscribers ---
        self.publisher_cmd = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.subscription_obstacle_turtle = self.create_subscription(
            TurtlePose, "/turtle2/pose", self.obstacle_pose_callback, 10
        )
        self.subscription_robot_turtle = self.create_subscription(
            TurtlePose, "/turtle1/pose", self.robot_pose_callback, 10
        )
        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )
        self.pub_potencial = self.create_publisher(
            Float32MultiArray, "/desvio_potencial", 10
        )

        # --- Estado ---
        self.obstacle_pose = None
        self.robot_pose = None
        self.last_joy_msg = None
        self.pass_through_mode = False
        self.prev_button_y = False

        self.v_max = 1.5  # Velocidade linear máxima
        self.kp_angular = 5.0  # Ganho de giro

        # --- Desvio potencial ---
        self.a = 1.0
        self.b = 1.0
        self.n = 4
        self.obstacle_gain = 8.0
        self.jacobian_dumping = 0.1

        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.potential_timer = self.create_timer(0.1, self.generate_potential_field)

        self.get_logger().info("Tartaruga obstáculo criada.")

    def obstacle_pose_callback(self, msg):
        self.obstacle_pose = [msg.x, msg.y, msg.theta]

    def robot_pose_callback(self, msg):
        self.robot_pose = [msg.x, msg.y, msg.theta]

    def joy_callback(self, msg):
        if msg.buttons[0] and not self.prev_button_y:
            self.pass_through_mode = not self.pass_through_mode
            status = "AUTÔNOMO" if self.pass_through_mode else "MANUAL"
            self.get_logger().info(f">>> MODO {status} ATIVADO")
        self.prev_button_y = bool(msg.buttons[0])
        self.last_joy_msg = msg

    def calculate_repulsive_velocity(
        self, robot_x_pos, robot_y_pos, obs_x_pos, obs_y_pos
    ):
        """Calcula a velocidade repulsiva com base na posição 2D do robô e
        do obstáculo. \n
        *Problema*: A tartaruga não desvia do obstáculo"""

        dx = robot_x_pos - obs_x_pos
        dy = robot_y_pos - obs_y_pos
        H = np.exp(-(dx**self.n) / self.a) * np.exp(-(dy**self.n) / self.b)

        potential_gradient_x = -H * self.n * (dx ** (self.n - 1)) / self.a
        potential_gradient_y = -H * self.n * (dy ** (self.n - 1)) / self.b

        obstacle_jacobian = np.array([[potential_gradient_x, potential_gradient_y]])
        denom = np.dot(obstacle_jacobian, obstacle_jacobian.T) + self.jacobian_dumping
        pseudo_inv_jacobian = obstacle_jacobian.T / denom
        repulsive_velocity = -pseudo_inv_jacobian * self.obstacle_gain * H
        return repulsive_velocity.flatten()

    def calculate_null_space_repulsive_velocity(
        self, robot_x_pos, robot_y_pos, obs_x_pos, obs_y_pos
    ):
        """
        Campo potencial híbrido em espaço nulo:
            - Jacobiana define a direção proibida (gradiente do potencial)
            - Projeção em espaço nulo gera evasão lateral
        """
        dx = robot_x_pos - obs_x_pos
        dy = robot_y_pos - obs_y_pos

        if np.hypot(dx, dy) > 1.5:
            return np.array([0.0, 0.0])  # Sem influência além de 1.5m

        H = np.exp(-(dx**self.n) / self.a) * np.exp(-(dy**self.n) / self.b)

        dH_dx = -H * self.n * (dx ** (self.n - 1)) / self.a
        dH_dy = -H * self.n * (dy ** (self.n - 1)) / self.b

        J = np.array([[dH_dx, dH_dy]])

        J_pseudo_inv = J.T / ((J @ J.T) + self.jacobian_dumping)

        I = np.eye(2)
        null_space_projector = I - (J_pseudo_inv @ J)

        z = np.array([-dH_dy, dH_dx])  # Vetor ortogonal ao gradiente
        z_normalized = z / (np.linalg.norm(z) + 1e-6)

        evasion_velocity = (
            self.obstacle_gain * H * (null_space_projector @ z_normalized)
        )
        return evasion_velocity.flatten()

    def generate_potential_field(self):
        """Gera o campo potencial com base na posição do robô e do obstáculo,
        e publica a velocidade repulsiva no tópico /desvio_potencial."""

        self.obstacles = []
        total_velocity = np.array([0.0, 0.0])

        if self.robot_pose and self.obstacle_pose:
            repulsive_vel = self.calculate_null_space_repulsive_velocity(
                self.robot_pose[0],
                self.robot_pose[1],
                self.obstacle_pose[0],
                self.obstacle_pose[1],
            )
            total_velocity += repulsive_vel
            self.obstacles.append(self.obstacle_pose)

        potential_msg = Float32MultiArray()
        potential_msg.data = [float(total_velocity[0]), float(total_velocity[1])]
        self.pub_potencial.publish(potential_msg)

    def control_loop(self):
        """Loop de controle que publica comandos de velocidade para a tartaruga robô
        com base no modo atual (manual ou autônomo)."""

        cmd = Twist()

        if not self.pass_through_mode:
            if self.last_joy_msg is None:
                return
            cmd.linear.x = self.last_joy_msg.axes[1] * self.v_max
            cmd.angular.z = self.last_joy_msg.axes[3] * self.kp_angular
            self.publisher_cmd.publish(cmd)
            return


def main():
    rclpy.init()
    node = ObstacleTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
