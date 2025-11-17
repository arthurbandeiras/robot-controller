import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy
)


def ms(ms_val):
    from rclpy.duration import Duration
    return Duration(seconds=0, nanoseconds=int(ms_val * 1e6))


class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control_differential')

        # QoS para comandos
        q_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS para joystick
        q_joy = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers e subscribers
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', q_cmd)
        self.subscription_joy = self.create_subscription(Joy, '/joy', self.joy_callback, q_joy)
        self.subscription_autonomo = self.create_subscription(Twist, '/robis/cmd_vel_autonomo', self.autonomo_callback, q_cmd)

        # Estado
        self.last_joy_msg = None
        self.autonomo_msg = None
        self.prev_button_y = False
        self.pass_through_mode = False
        self.prev_axis_8 = 0
        self.k_speed = 0.2

        # Timer de publicação 60 Hz
        self.timer = self.create_timer(1/60.0, self.publish_command)

        self.get_logger().info("JoystickControl (modo diferencial) iniciado.")

    def joy_callback(self, msg: Joy):
        self.last_joy_msg = msg

    def autonomo_callback(self, msg: Twist):
        self.autonomo_msg = msg

    def publish_command(self):
        # Sem joystick -> não faz nada
        if not self.last_joy_msg:
            return

        axes = self.last_joy_msg.axes
        btns = self.last_joy_msg.buttons

        # Alternância pass-through (botão Y)
        if btns[3] and not self.prev_button_y:
            self.pass_through_mode = not self.pass_through_mode
            modo = "pass-through" if self.pass_through_mode else "manual"
            self.get_logger().info(f"Modo alterado para: {modo}")
        self.prev_button_y = btns[3]

        # Se em pass-through -> repassa mensagem de controle autônomo
        if self.pass_through_mode:
            if self.autonomo_msg:
                self.publisher_cmd.publish(self.autonomo_msg)
            self.last_joy_msg = None
            return

        # Controle manual
        # Eixos: left stick vertical (axes[1]) = linear x
        #        right stick horizontal (axes[3]) = angular z
        linear_x = np.clip(axes[1], -1, 1) * self.k_speed
        angular_z = np.clip(axes[3], -1, 1) * self.k_speed

        # Ajuste de ganho com D-Pad (eixo 8 vertical)
        axis_8 = int(np.sign(axes[7])) if len(axes) > 7 else 0
        if axis_8 and axis_8 != self.prev_axis_8:
            self.k_speed = np.clip(self.k_speed + axis_8 * 0.1, 0.0, 5.0)
            self.get_logger().info(f"k_speed = {self.k_speed:.1f}")
        self.prev_axis_8 = axis_8

        # Publica comando
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.publisher_cmd.publish(cmd)

        self.last_joy_msg = None


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
