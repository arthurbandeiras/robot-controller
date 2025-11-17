import numpy as np
from .ros_pb2 import ROSMessage
from google.protobuf.struct_pb2 import Struct

from is_msgs.image_pb2 import HumanKeypoints as HKP, ObjectAnnotation

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Header, ColorRGBA
from custom_message.msg import SkeletonList, Skeleton, Joint


def create_ros2_skeleton_list(
    is_skeletons: list[ObjectAnnotation], frame_id: str = "Map"
) -> SkeletonList:
    """
    Cria uma mensagem ROS 2 SkeletonList a partir de uma lista de ObjectAnnotations (esqueletos).

    Retorna: tutorial_interfaces.msg.SkeletonList
    """

    skeleton_list_msg = SkeletonList()

    ros_skeletons = []

    for forced_id, is_skeleton in enumerate(is_skeletons):

        skeleton_msg = Skeleton()

        final_id = forced_id if is_skeleton.id == 0 else is_skeleton.id
        skeleton_msg.skeleton_id = final_id

        joint_messages = []

        for kp in is_skeleton.keypoints:

            joint_msg = Joint()
            joint_msg.id = kp.id

            position_msg = Point(x=kp.position.x, y=kp.position.y, z=kp.position.z)
            joint_msg.position = position_msg
            joint_messages.append(joint_msg)

        skeleton_msg.joints = joint_messages
        ros_skeletons.append(skeleton_msg)

    skeleton_list_msg.skeletons = ros_skeletons

    return skeleton_list_msg


def create_ros2_marker_array(
    skeleton: ObjectAnnotation, forced_id: int, frame_id: str = "Map"
) -> MarkerArray:
    """
    Cria uma mensagem ROS 2 MarkerArray contendo SPHERES para as juntas e TEXT_VIEW_FACING
    para os ID's das juntas de um único esqueleto.

    Retorna: visualization_msgs.msg.MarkerArray
    """

    marker_array = MarkerArray()

    ns_id = forced_id if skeleton.id == 0 else skeleton.id

    # ------------------------------------------------
    # 1. Marcador de Juntas (Esferas) - Marker.SPHERE
    # ------------------------------------------------

    joint_points = []
    for kp in skeleton.keypoints:
        joint_points.append(Point(x=kp.position.x, y=kp.position.y, z=kp.position.z))

    sphere_marker = Marker(
        header=Header(frame_id=frame_id),
        ns=f"skeleton_joints_id_{ns_id}",
        id=1000,
        type=Marker.SPHERE_LIST,
        action=Marker.ADD,
        scale=Vector3(x=0.08, y=0.08, z=0.08),
        color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
        points=joint_points,
    )
    marker_array.markers.append(sphere_marker)

    # ------------------------------------------------
    # 2. Marcadores de Texto (ID's das Juntas)
    # ------------------------------------------------

    for kp in skeleton.keypoints:
        text_marker = Marker(
            header=Header(frame_id=frame_id),
            ns=f"skeleton_labels_id_{ns_id}",
            id=kp.id,
            type=Marker.TEXT_VIEW_FACING,
            action=Marker.ADD,
            pose=Pose(
                position=Point(x=kp.position.x, y=kp.position.y, z=kp.position.z)
            ),
            scale=Vector3(z=0.15),
            color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
            text=str(kp.id),
        )
        marker_array.markers.append(text_marker)

    return marker_array


def create_is_ros_message(skeleton, forced_id, frame_id: str = "Map"):
    """
    Cria uma mensagem ROSMarkerArray contendo SPHERES para as juntas e TEXT_VIEW_FACING
    para os ID's das juntas de um único esqueleto.

    skeleton = skeletons.objects.

    forced_id = usado caso não tenha id no skeleton.

    frame_id informa ao Rviz e ao sistema ROS: "Estes pontos X, Y, Z são relativos a este ponto fixo no espaço."
    """

    joint_points = []
    text_markers = []

    ns_id = forced_id if skeleton.id == 0 else skeleton.id

    for kp in skeleton.keypoints:
        pos_dict = {"x": kp.position.x, "y": kp.position.y, "z": kp.position.z}

        joint_points.append(pos_dict)

        text_marker = {
            "header": {"frame_id": frame_id},
            "ns": f"skeleton_labels_id_{ns_id}",
            "id": kp.id,
            "type": 9,  # Marker.TEXT_VIEW_FACING
            "action": 0,  # Marker.ADD
            "pose": {"position": pos_dict},
            "scale": {"z": 0.15},
            "color": {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0},
            "text": str(kp.id),
        }
        text_markers.append(text_marker)

    marker_array_dict = {
        "markers": [
            # --- Marcador 1: Juntas (Esferas) ---
            {
                "header": {"frame_id": frame_id},
                "ns": f"skeleton_joints_id_{ns_id}",
                "id": 1000,
                "type": 2,  # Marker.SPHERE
                "action": 0,  # Marker.ADD
                "scale": {"x": 0.08, "y": 0.08, "z": 0.08},
                "color": {"r": 1.0, "g": 0, "b": 0, "a": 1.0},
                "points": joint_points,
            }
        ]
        + text_markers
    }

    print(marker_array_dict)

    marker_array_struct = Struct()
    marker_array_struct.update(marker_array_dict)

    ros_message = ROSMessage(content=marker_array_struct)
    ros_message.type = "visualization_msgs/msg/MarkerArray"

    return ros_message
