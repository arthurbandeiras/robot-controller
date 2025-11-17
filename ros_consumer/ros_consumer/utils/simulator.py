try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    rclpy = None
    Node = object  # Define Node como um objeto genérico se rclpy não estiver disponível
import random
import math
from typing import List, Dict, Tuple

# Importações de mensagens ROS 2
# Assumindo que o usuário tem o pacote 'custom_message' e 'geometry_msgs'
try:
    from custom_message.msg import SkeletonList, Skeleton, Joint
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header
except ImportError:
    # Fallback para simulação sem ROS 2 se as mensagens não estiverem disponíveis
    # No ambiente de teste, usaremos as classes definidas abaixo para simular a estrutura
    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    class Joint:
        def __init__(self, id=0, position=None):
            self.id = id
            self.position = position if position is not None else Point()

    class Skeleton:
        def __init__(self, skeleton_id=0, joints=None):
            self.skeleton_id = skeleton_id
            self.joints = joints if joints is not None else []

    class SkeletonList:
        def __init__(self, skeletons=None):
            self.skeletons = skeletons if skeletons is not None else []
            self.header = Header()  # Adicionando Header para compatibilidade com ROS 2

    class Header:
        def __init__(self):
            self.frame_id = "Map"

    # Definindo classes mock para Node e rclpy para permitir testes fora do ambiente ROS 2
    class Node:
        def __init__(self, name):
            pass

        def get_logger(self):
            return self

        def info(self, msg):
            print(f"[INFO] {msg}")

        def debug(self, msg):
            print(f"[DEBUG] {msg}")

        def error(self, msg):
            print(f"[ERROR] {msg}")

        def create_publisher(self, msg_type, topic, qos):
            return self

        def publish(self, msg):
            pass

        def create_timer(self, period, callback):
            pass

        def destroy_node(self):
            pass

    class rclpy:
        def init(args=None):
            pass

        def spin(node):
            pass

        def shutdown():
            pass


# --- Constantes e Estrutura do Esqueleto ---

# IDs das 17 juntas do COCO
JOINT_IDS = {
    "NOSE": 0,
    "LEFT_EYE": 1,
    "RIGHT_EYE": 2,
    "LEFT_EAR": 3,
    "RIGHT_EAR": 4,
    "LEFT_SHOULDER": 5,
    "RIGHT_SHOULDER": 6,
    "LEFT_ELBOW": 7,
    "RIGHT_ELBOW": 8,
    "LEFT_WRIST": 9,
    "RIGHT_WRIST": 10,
    "LEFT_HIP": 11,
    "RIGHT_HIP": 12,
    "LEFT_KNEE": 13,
    "RIGHT_KNEE": 14,
    "RIGHT_ANKLE": 15,
    "LEFT_ANKLE": 16,
}

# Relações de parentesco para simular o movimento de um esqueleto real
# A posição de uma junta é relativa à sua junta pai.
# Usaremos o quadril central (média dos quadris) como ponto de referência (root).
# Estrutura: {ID_FILHO: ID_PAI}
# Usaremos 100 como ID para o "Centro do Quadril" (Root)
JOINT_PARENTS = {
    JOINT_IDS["NOSE"]: JOINT_IDS["RIGHT_EAR"],  # Simplificação: nariz ligado à orelha
    JOINT_IDS["LEFT_EYE"]: JOINT_IDS["NOSE"],
    JOINT_IDS["RIGHT_EYE"]: JOINT_IDS["NOSE"],
    JOINT_IDS["LEFT_EAR"]: JOINT_IDS["LEFT_SHOULDER"],
    JOINT_IDS["RIGHT_EAR"]: JOINT_IDS["RIGHT_SHOULDER"],
    JOINT_IDS["LEFT_SHOULDER"]: 100,  # Centro do Quadril (Root)
    JOINT_IDS["RIGHT_SHOULDER"]: 100,  # Centro do Quadril (Root)
    JOINT_IDS["LEFT_ELBOW"]: JOINT_IDS["LEFT_SHOULDER"],
    JOINT_IDS["RIGHT_ELBOW"]: JOINT_IDS["RIGHT_SHOULDER"],
    JOINT_IDS["LEFT_WRIST"]: JOINT_IDS["LEFT_ELBOW"],
    JOINT_IDS["RIGHT_WRIST"]: JOINT_IDS["RIGHT_ELBOW"],
    JOINT_IDS["LEFT_HIP"]: 100,  # Centro do Quadril (Root)
    JOINT_IDS["RIGHT_HIP"]: 100,  # Centro do Quadril (Root)
    JOINT_IDS["LEFT_KNEE"]: JOINT_IDS["LEFT_HIP"],
    JOINT_IDS["RIGHT_KNEE"]: JOINT_IDS["RIGHT_HIP"],
    JOINT_IDS["LEFT_ANKLE"]: JOINT_IDS["LEFT_KNEE"],
    JOINT_IDS["RIGHT_ANKLE"]: JOINT_IDS["RIGHT_KNEE"],
}

# Posições relativas (offsets) de um esqueleto em "T-pose" (valores arbitrários em metros)
# Estes são os offsets da junta em relação ao seu PARENT.
T_POSE_OFFSETS = {
    JOINT_IDS["NOSE"]: (0.0, 0.0, 0.1),
    JOINT_IDS["LEFT_EYE"]: (0.05, 0.0, 0.0),
    JOINT_IDS["RIGHT_EYE"]: (-0.05, 0.0, 0.0),
    JOINT_IDS["LEFT_EAR"]: (0.0, 0.0, 0.05),
    JOINT_IDS["RIGHT_EAR"]: (0.0, 0.0, 0.05),
    JOINT_IDS["LEFT_SHOULDER"]: (0.15, 0.0, 0.2),
    JOINT_IDS["RIGHT_SHOULDER"]: (-0.15, 0.0, 0.2),
    JOINT_IDS["LEFT_ELBOW"]: (0.2, 0.0, 0.0),
    JOINT_IDS["RIGHT_ELBOW"]: (-0.2, 0.0, 0.0),
    JOINT_IDS["LEFT_WRIST"]: (0.2, 0.0, 0.0),
    JOINT_IDS["RIGHT_WRIST"]: (-0.2, 0.0, 0.0),
    JOINT_IDS["LEFT_HIP"]: (0.1, 0.0, -0.2),
    JOINT_IDS["RIGHT_HIP"]: (-0.1, 0.0, -0.2),
    JOINT_IDS["LEFT_KNEE"]: (0.0, 0.0, -0.4),
    JOINT_IDS["RIGHT_KNEE"]: (0.0, 0.0, -0.4),
    JOINT_IDS["LEFT_ANKLE"]: (0.0, 0.0, -0.4),
    JOINT_IDS["RIGHT_ANKLE"]: (0.0, 0.0, -0.4),
    100: (0.0, 0.0, 0.0),  # Root (Centro do Quadril)
}

# --- Lógica de Suavização (Filtro de Primeira Ordem) ---


class FirstOrderFilter:
    """
    Implementa um filtro de primeira ordem (low-pass filter) para suavizar
    a transição entre o valor atual e o valor alvo.
    """

    def __init__(self, initial_value: float, alpha: float):
        """
        :param initial_value: O valor inicial do filtro.
        :param alpha: O fator de suavização (0 < alpha < 1).
                      alpha = 1 - exp(-dt / tau), onde tau é a constante de tempo.
                      Um alpha menor resulta em mais suavização.
        """
        self.current_value = initial_value
        self.alpha = alpha

    def update(self, target_value: float) -> float:
        """
        Atualiza o valor do filtro em direção ao valor alvo.
        """
        self.current_value = (
            self.alpha * target_value + (1.0 - self.alpha) * self.current_value
        )
        return self.current_value


# --- Classe de Simulação do Esqueleto ---


class SimulatedSkeleton:
    """
    Representa um único esqueleto simulado com movimento aleatório e suave.
    """

    def __init__(
        self, skeleton_id: int, alpha: float, bounds: Dict[str, Tuple[float, float]]
    ):
        self.skeleton_id = skeleton_id
        self.bounds = bounds
        self.alpha = alpha

        # 1. Posição do Root (Centro do Quadril)
        # O Root é o ponto central que define a posição do esqueleto no mundo.
        initial_x = random.uniform(bounds["X"][0], bounds["X"][1])
        initial_y = random.uniform(bounds["Y"][0], bounds["Y"][1])
        initial_z = random.uniform(bounds["Z"][0], bounds["Z"][1])

        self.root_position = Point()
        self.root_position.x = initial_x
        self.root_position.y = initial_y
        self.root_position.z = initial_z
        self.root_target = Point()
        self.root_target.x = initial_x
        self.root_target.y = initial_y
        self.root_target.z = initial_z

        # Filtros para suavizar o movimento do Root
        self.filter_x = FirstOrderFilter(initial_x, alpha)
        self.filter_y = FirstOrderFilter(initial_y, alpha)
        self.filter_z = FirstOrderFilter(initial_z, alpha)

        # 2. Posições Relativas das Juntas (Simulação de Pose)
        # Para simular um "andar" realista, vamos variar as posições relativas (offsets)
        # de algumas juntas (braços e pernas) em relação aos seus pais.
        self.joint_offsets: Dict[int, Point] = {}
        self.joint_offset_targets: Dict[int, Point] = {}
        self.joint_offset_filters: Dict[
            int, Tuple[FirstOrderFilter, FirstOrderFilter, FirstOrderFilter]
        ] = {}

        # Inicializa os offsets e filtros para as juntas que terão variação de pose
        self.variable_joints = [
            JOINT_IDS["LEFT_ELBOW"],
            JOINT_IDS["RIGHT_ELBOW"],
            JOINT_IDS["LEFT_KNEE"],
            JOINT_IDS["RIGHT_KNEE"],
            JOINT_IDS["LEFT_WRIST"],
            JOINT_IDS["RIGHT_WRIST"],
            JOINT_IDS["LEFT_ANKLE"],
            JOINT_IDS["RIGHT_ANKLE"],
        ]

        for joint_id in JOINT_IDS.values():
            if joint_id in self.variable_joints:
                # Inicializa com o offset da T-pose
                ox, oy, oz = T_POSE_OFFSETS[joint_id]
                initial_offset = Point()
                initial_offset.x = ox
                initial_offset.y = oy
                initial_offset.z = oz
                self.joint_offsets[joint_id] = initial_offset
                self.joint_offset_targets[joint_id] = initial_offset

                # Cria filtros para cada componente (x, y, z) do offset
                self.joint_offset_filters[joint_id] = (
                    FirstOrderFilter(ox, alpha),
                    FirstOrderFilter(oy, alpha),
                    FirstOrderFilter(oz, alpha),
                )

    def _generate_new_root_target(self):
        """
        Gera um novo ponto alvo aleatório para o Root dentro dos limites.
        """
        self.root_target.x = random.uniform(self.bounds["X"][0], self.bounds["X"][1])
        self.root_target.y = random.uniform(self.bounds["Y"][0], self.bounds["Y"][1])
        self.root_target.z = random.uniform(self.bounds["Z"][0], self.bounds["Z"][1])

    def _generate_new_pose_target(self):
        """
        Gera novos alvos de pose (offsets relativos) para simular o movimento.
        """
        # Simulação de movimento de "andar"
        # A cada passo, as pernas e braços variam um pouco.

        # Variação de pose (em metros)
        pose_variance = 0.15

        for joint_id in self.variable_joints:
            ox, oy, oz = T_POSE_OFFSETS[joint_id]

            # Adiciona uma variação aleatória ao offset da T-pose
            target_x = ox + random.uniform(-pose_variance, pose_variance)
            target_y = oy + random.uniform(-pose_variance, pose_variance)
            target_z = oz + random.uniform(-pose_variance, pose_variance)

            target_point = Point()
            target_point.x = target_x
            target_point.y = target_y
            target_point.z = target_z
            self.joint_offset_targets[joint_id] = target_point

    def update(self):
        """
        Atualiza a posição do esqueleto.
        """
        # 1. Atualiza o alvo do Root periodicamente
        if random.random() < 0.05:  # 5% de chance de mudar o alvo a cada atualização
            self._generate_new_root_target()

        # 2. Atualiza o alvo da Pose periodicamente
        if random.random() < 0.1:  # 10% de chance de mudar a pose a cada atualização
            self._generate_new_pose_target()

        # 3. Aplica o filtro de primeira ordem para suavizar o movimento do Root
        self.root_position.x = self.filter_x.update(self.root_target.x)
        self.root_position.y = self.filter_y.update(self.root_target.y)
        self.root_position.z = self.filter_z.update(self.root_target.z)

        # 4. Aplica o filtro de primeira ordem para suavizar a variação da Pose
        for joint_id in self.variable_joints:
            target = self.joint_offset_targets[joint_id]
            filters = self.joint_offset_filters[joint_id]

            # Atualiza o offset atual com o filtro
            self.joint_offsets[joint_id].x = filters[0].update(target.x)
            self.joint_offsets[joint_id].y = filters[1].update(target.y)
            self.joint_offsets[joint_id].z = filters[2].update(target.z)

    def get_skeleton_msg(self) -> Skeleton:
        """
        Calcula as posições absolutas das juntas e retorna a mensagem Skeleton ROS 2.
        """
        skeleton_msg = Skeleton(skeleton_id=self.skeleton_id)

        # Dicionário para armazenar as posições absolutas calculadas
        absolute_positions: Dict[int, Point] = {100: self.root_position}

        # Ordem de cálculo: do Root para as extremidades
        # A ordem das chaves em JOINT_PARENTS garante que o pai seja calculado antes do filho

        # Juntas que dependem do Root (Centro do Quadril)
        root_children = [
            JOINT_IDS["LEFT_SHOULDER"],
            JOINT_IDS["RIGHT_SHOULDER"],
            JOINT_IDS["LEFT_HIP"],
            JOINT_IDS["RIGHT_HIP"],
        ]

        # Lista de todas as juntas, ordenada para garantir que os pais sejam processados primeiro
        ordered_joints = sorted(JOINT_IDS.values())

        # Processa as juntas em ordem hierárquica
        for joint_id in ordered_joints:
            parent_id = JOINT_PARENTS.get(joint_id)

            # Se o pai ainda não foi calculado, pula (isso deve acontecer apenas se a ordem estiver errada)
            if parent_id not in absolute_positions:
                continue

            parent_pos = absolute_positions[parent_id]

            # Offset da junta em relação ao pai
            if joint_id in self.joint_offsets:
                # Usa o offset dinâmico (para braços e pernas)
                offset = self.joint_offsets[joint_id]
            else:
                # Usa o offset estático da T-pose (para tronco e cabeça)
                ox, oy, oz = T_POSE_OFFSETS[joint_id]
                offset = Point()
                offset.x = ox
                offset.y = oy
                offset.z = oz

            # Posição absoluta = Posição do Pai + Offset
            abs_pos = Point()
            abs_pos.x = parent_pos.x + offset.x
            abs_pos.y = parent_pos.y + offset.y
            abs_pos.z = parent_pos.z + offset.z

            absolute_positions[joint_id] = abs_pos

            # Cria a mensagem Joint
            joint_msg = Joint(id=joint_id, position=abs_pos)
            skeleton_msg.joints.append(joint_msg)

        return skeleton_msg


# --- Nó ROS 2 de Simulação ---


class SkeletonSimulatorNode(Node):
    """
    Nó ROS 2 que simula a movimentação de dois esqueletos.
    """

    def __init__(self):
        super().__init__("skeleton_simulator_node")

        # Parâmetros de Simulação
        self.ROS_FRAME_ID = "Map"
        self.NUM_SKELETONS = 2
        self.PUBLISH_RATE = 0.1  # 10 Hz (a cada 0.1s)

        # Limites de Movimento (fornecidos pelo usuário)
        self.BOUNDS = {"X": (-2.0, 2.0), "Y": (-2.0, 2.0), "Z": (0.0, 2.0)}

        # Fator de Suavização (Alpha) para o filtro de primeira ordem
        # alpha = 1 - exp(-dt / tau)
        # Para dt=0.1s e uma constante de tempo tau=0.5s (movimento suave), alpha ~ 0.18
        # Usaremos um valor um pouco menor para um movimento mais suave.
        self.ALPHA = 0.02

        # Inicializa os esqueletos simulados
        self.skeletons: List[SimulatedSkeleton] = []
        for i in range(self.NUM_SKELETONS):
            self.skeletons.append(
                SimulatedSkeleton(
                    skeleton_id=i + 1, alpha=self.ALPHA, bounds=self.BOUNDS  # IDs 1 e 2
                )
            )

        # Configuração do Publisher
        self.publisher_ = self.create_publisher(SkeletonList, "/skeleton_list", 10)

        # Configuração do Timer
        self.timer = self.create_timer(self.PUBLISH_RATE, self.timer_callback)
        self.get_logger().info(
            f"Nó SkeletonSimulatorNode iniciado. Publicando a {1/self.PUBLISH_RATE} Hz."
        )

    def timer_callback(self):
        """Função chamada pelo timer para atualizar e publicar os esqueletos."""
        try:
            skeleton_list_msg = SkeletonList()

            # Adiciona o cabeçalho (Header)
            header = Header()
            header.frame_id = self.ROS_FRAME_ID
            # O timestamp será preenchido automaticamente pelo ROS 2, mas o simulador
            # precisa de um campo de cabeçalho para ser compatível com a estrutura
            # se estiver sendo usado fora do ROS 2, o campo 'header' precisa ser adicionado
            # à classe SkeletonList mock, como feito acima.
            # No ROS 2 real, o SkeletonList provavelmente não tem um header, mas a
            # mensagem que o contém (como ObjectAnnotations no seu código original) teria.
            # Para simplificar, assumimos que o SkeletonList tem um header, ou que o
            # nó de destino não se importa.

            for skeleton in self.skeletons:
                # 1. Atualiza a posição e pose do esqueleto
                skeleton.update()

                # 2. Gera a mensagem ROS 2
                skeleton_msg = skeleton.get_skeleton_msg()
                skeleton_list_msg.skeletons.append(skeleton_msg)

            # 3. Publica a mensagem
            self.publisher_.publish(skeleton_list_msg)

            self.get_logger().debug(
                f"Publicada mensagem SkeletonList com {len(skeleton_list_msg.skeletons)} esqueletos."
            )

        except Exception as e:
            self.get_logger().error(f"Erro durante o processamento (callback): {e}")


def main(args=None):
    if rclpy is not None:
        rclpy.init(args=args)
        skeleton_simulator_node = SkeletonSimulatorNode()
        try:
            rclpy.spin(skeleton_simulator_node)
        except KeyboardInterrupt:
            pass
        finally:
            skeleton_simulator_node.destroy_node()
            rclpy.shutdown()
    else:
        # Se rclpy não estiver disponível, o nó não pode ser executado.
        # O usuário precisará executar em um ambiente ROS 2.
        print(
            "O módulo 'rclpy' não foi encontrado. Por favor, execute este nó em um ambiente ROS 2."
        )


if __name__ == "__main__":
    # Execução de teste da lógica de simulação (sem ROS 2 real)
    if rclpy is None:
        print("Executando lógica de simulação em modo de teste (sem ROS 2 real).")
        node = SkeletonSimulatorNode()
        # Simula 10 ciclos de atualização e publicação
        for i in range(10):
            node.timer_callback()
            # Imprime a posição do Root do primeiro esqueleto para validação
            root_pos = node.skeletons[0].root_position
            print(
                f"Ciclo {i+1}: Root Skeleto 1: X={root_pos.x:.4f}, Y={root_pos.y:.4f}, Z={root_pos.z:.4f}"
            )
            # Simula o tempo de espera
            import time

            time.sleep(node.PUBLISH_RATE)
    else:
        main()
