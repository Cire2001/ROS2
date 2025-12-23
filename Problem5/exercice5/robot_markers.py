# exercice5/robot_markers.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from exercice5.vec2d import Vec2D
import math

class RobotMarkers(Node):
    def __init__(self):
        super().__init__('robot_markers')

        # Nombre de robots
        self.num_robots = 3  # Modifié de 5 à 3

        # Positions des robots
        self.robot_positions = {}
        self.robot_orientations = {}

        # Couleurs des robots (bleu avec différentes nuances)
        self.robot_colors = {
            1: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),    # Bleu
            2: ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0),    # Bleu ciel
            3: ColorRGBA(r=0.0, g=0.8, b=1.0, a=1.0),    # Bleu turquoise
            4: ColorRGBA(r=0.3, g=0.3, b=1.0, a=1.0),    # Bleu violet
            5: ColorRGBA(r=0.0, g=0.2, b=0.8, a=1.0)    # Bleu marine
        }

        # Abonnements aux positions des robots
        for i in range(1, self.num_robots + 1):
            self.create_subscription(
                PoseStamped,
                f'/robot_{i}/current_position',
                lambda msg, robot_id=i: self.robot_position_callback(msg, robot_id),
                10)

        # Publication des marqueurs
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/robot_markers',
            10)

        # Timer pour publier les marqueurs
        self.timer = self.create_timer(0.1, self.publish_markers)

        self.get_logger().info('Robot Markers node initialized')

    def robot_position_callback(self, msg, robot_id):
        """Callback pour la position d'un robot"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_positions[robot_id] = Vec2D(x, y)

        # Extraire l'orientation (quaternion -> angle)
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Conversion simplifiée du quaternion en angle yaw
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.robot_orientations[robot_id] = yaw

    def publish_markers(self):
        """Publie les marqueurs pour tous les robots"""
        marker_array = MarkerArray()

        for robot_id, position in self.robot_positions.items():
            # Créer un marqueur pour le corps du robot
            body_marker = Marker()
            body_marker.header.frame_id = "map"
            body_marker.header.stamp = self.get_clock().now().to_msg()
            body_marker.ns = f"robot_{robot_id}"
            body_marker.id = robot_id
            body_marker.type = Marker.CYLINDER
            body_marker.action = Marker.ADD

            # Position
            body_marker.pose.position.x = position.x
            body_marker.pose.position.y = position.y
            body_marker.pose.position.z = 0.5  # Légèrement au-dessus du sol

            # Orientation (identité = pas de rotation)
            body_marker.pose.orientation.x = 0.0
            body_marker.pose.orientation.y = 0.0
            body_marker.pose.orientation.z = 0.0
            body_marker.pose.orientation.w = 1.0

            # Échelle (taille du robot)
            body_marker.scale.x = 2.0  # Diamètre
            body_marker.scale.y = 2.0  # Diamètre
            body_marker.scale.z = 1.0  # Hauteur

            # Couleur
            body_marker.color = self.robot_colors[robot_id]

            # Durée de vie (0 = forever)
            body_marker.lifetime.sec = 0
            body_marker.lifetime.nanosec = 0

            marker_array.markers.append(body_marker)

            # Créer un marqueur pour la direction (flèche)
            if robot_id in self.robot_orientations:
                direction_marker = Marker()
                direction_marker.header.frame_id = "map"
                direction_marker.header.stamp = self.get_clock().now().to_msg()
                direction_marker.ns = f"robot_{robot_id}_direction"
                direction_marker.id = robot_id + 100  # ID différent pour éviter les conflits
                direction_marker.type = Marker.ARROW
                direction_marker.action = Marker.ADD

                # Position
                direction_marker.pose.position.x = position.x
                direction_marker.pose.position.y = position.y
                direction_marker.pose.position.z = 0.5  # Même hauteur que le corps

                # Orientation (utiliser l'orientation du robot)
                yaw = self.robot_orientations[robot_id]
                direction_marker.pose.orientation.x = 0.0
                direction_marker.pose.orientation.y = 0.0
                direction_marker.pose.orientation.z = math.sin(yaw / 2.0)
                direction_marker.pose.orientation.w = math.cos(yaw / 2.0)

                # Échelle (taille de la flèche)
                direction_marker.scale.x = 3.0  # Longueur
                direction_marker.scale.y = 0.5  # Largeur
                direction_marker.scale.z = 0.5  # Hauteur

                # Couleur (plus claire que le corps)
                color = ColorRGBA()
                color.r = min(1.0, self.robot_colors[robot_id].r + 0.2)
                color.g = min(1.0, self.robot_colors[robot_id].g + 0.2)
                color.b = min(1.0, self.robot_colors[robot_id].b + 0.2)
                color.a = 1.0
                direction_marker.color = color

                # Durée de vie (0 = forever)
                direction_marker.lifetime.sec = 0
                direction_marker.lifetime.nanosec = 0

                marker_array.markers.append(direction_marker)

        # Publier tous les marqueurs
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMarkers()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
