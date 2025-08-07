import rclpy
from rclpy.node import Node
import subprocess
import importlib
from rclpy.qos import QoSProfile
import time

# # # from pprint import pformat
# # # from rclpy.serialization import serialize_message
# # # from rosidl_runtime_py.utilities import message_to_ordereddict


# # # def pretty_print_msg(msg, max_list_items=5):
# # #     """
# # #     Affiche proprement un message ROS 2, tronque les grandes listes.
# # #     """
# # #     try:
# # #         # Convertir le message en OrderedDict
# # #         data = message_to_ordereddict(msg)

# # #         # Fonction pour tronquer les longues listes récursivement
# # #         def truncate(obj):
# # #             if isinstance(obj, list):
# # #                 if len(obj) > max_list_items:
# # #                     return obj[:max_list_items] + [f"... (+{len(obj) - max_list_items} items)"]
# # #                 else:
# # #                     return [truncate(x) for x in obj]
# # #             elif isinstance(obj, dict):
# # #                 return {k: truncate(v) for k, v in obj.items()}
# # #             else:
# # #                 return obj

# # #         clean_data = truncate(data)
# # #         return pformat(clean_data, indent=2, width=80, compact=True)
# # #     except Exception as e:
# # #         return f"[Erreur de formatage]: {e}"

# # from pprint import pformat

# # def pretty_print_msg(msg, max_list_items=5):
# #     def truncate(obj):
# #         if isinstance(obj, list):
# #             if len(obj) > max_list_items:
# #                 return obj[:max_list_items] + [f"... (+{len(obj) - max_list_items} items)"]
# #             else:
# #                 return [truncate(x) for x in obj]
# #         elif hasattr(obj, '__slots__'):  # message ROS2 ou champ composé
# #             return {slot: truncate(getattr(obj, slot)) for slot in obj.__slots__}
# #         elif isinstance(obj, dict):
# #             return {k: truncate(v) for k, v in obj.items()}
# #         else:
# #             return obj

# #     try:
# #         data = truncate(msg)
# #         return pformat(data, indent=2, width=80, compact=True)
# #     except Exception as e:
# #         return f"[Erreur de formatage]: {e}"


# from pprint import pformat

# def pretty_print_msg(msg, max_list_items=5, max_depth=4):
#     def truncate(obj, depth=0):
#         if depth > max_depth:
#             return '...'

#         if isinstance(obj, list):
#             length = len(obj)
#             if length > max_list_items:
#                 preview = [truncate(x, depth + 1) for x in obj[:max_list_items]]
#                 preview.append(f"... (+{length - max_list_items} items)")
#                 return preview
#             else:
#                 return [truncate(x, depth + 1) for x in obj]
#         elif isinstance(obj, tuple):
#             return tuple(truncate(list(obj), depth))
#         elif isinstance(obj, dict):
#             return {k: truncate(v, depth + 1) for k, v in obj.items()}
#         elif hasattr(obj, '__slots__'):  # ROS message object
#             return {slot: truncate(getattr(obj, slot), depth + 1) for slot in obj.__slots__}
#         else:
#             return obj

#     try:
#         data = truncate(msg)
#         return pformat(data, indent=2, width=100, compact=True)
#     except Exception as e:
#         return f"[Erreur de formatage]: {e}"


def flatten_ros_message(msg, prefix=''):
    """Flatte récursivement un message ROS en un dict champ:valeur (chaîne)."""
    flat = {}

    if hasattr(msg, '__slots__'):
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            key = f"{prefix}.{slot}" if prefix else slot
            flat.update(flatten_ros_message(val, key))
    elif isinstance(msg, (list, tuple)):
        flat[prefix] = f"[{len(msg)} items]" if len(msg) > 5 else str(msg)
    else:
        flat[prefix] = str(msg)

    return flat


def pretty_print_msg(msg, max_char=100):
    flat = flatten_ros_message(msg)
    lines = []

    for key, val in flat.items():
        if len(val) > max_char:
            val = val[:max_char - 5] + " ..."

        lines.append(f"{key}: {val}")
    summary = f"[Résumé: {len(flat)} champs affichés]"
    return '\n'.join(lines + [summary])

    # return '\n'.join(lines)


def get_topics_and_types():
    try:
        result = subprocess.run(['ros2', 'topic', 'list', '-t'],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                check=True)
        output = result.stdout.decode().strip().split('\n')
        topics = []
        print("found those topics: ")
        for e in output:
            if e.split(' ')[0] in FILTERED_TOPICS:
                print(f"ignoring : {e}")
            else:
                print(e)
        print("sleeping 10 sec for UX")
        time.sleep(10)

        for line in output:
            if not line.strip():
                continue
            if '[' in line and ']' in line:
                name = line.split('[')[0].strip()
                type_str = line.split('[')[1].split(']')[0].strip()
                topics.append((name, type_str))
        return topics
    except subprocess.CalledProcessError as e:
        print("Erreur lors de l'exécution de 'ros2 topic list -t':", e.stderr.decode())
        return []


def import_message_type(type_str):
    """Ex: 'std_msgs/msg/String' -> <class 'std_msgs.msg.String'>"""
    try:
        package, _, msg_type = type_str.partition('/msg/')
        module = importlib.import_module(f'{package}.msg')
        return getattr(module, msg_type)
    except (ImportError, AttributeError) as e:
        print(f"Impossible d'importer le type {type_str}: {e}")
        return None

FILTERED_TOPICS = [
    "/clock",
    "/rosout",
    "/performance_metrics",
    "/tf_static",
    "/tello_7/joy",
    "/tello_7/cmd_vel",
    "/tello_7/imu_plugin_7/out",
    "/tello_7/imu/data_raw",
    "/tello_7/tello_7/image_raw",
    "/tello_7/tello_7/camera_info"
]


class DynamicSubscriberNode(Node):
    def __init__(self):
        super().__init__('dynamic_subscriber_node')
        self.subscribers = []

        qos_profile = QoSProfile(depth=10)
        topics = get_topics_and_types()

        for topic_name, type_str in topics:
            if topic_name in FILTERED_TOPICS:
                self.get_logger().info(f"ignoring : {topic_name} [{type_str}]")
                continue

            msg_type = import_message_type(type_str)
            if msg_type is None:
                self.get_logger().warning(f"Type non supporté pour le topic {topic_name}: {type_str}")
                continue

            self.get_logger().info(f"Créer un subscriber sur {topic_name} [{type_str}]")
            callback = self.create_callback(topic_name)
            sub = self.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos_profile
            )
            self.subscribers.append(sub)

    def create_callback(self, topic_name):
        def callback(msg):
            pretty = pretty_print_msg(msg)
            self.get_logger().info(f"[{topic_name}]\n{pretty}")
        return callback


def main(args=None):
    rclpy.init(args=args)
    node = DynamicSubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Arrêté par l'utilisateur.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


