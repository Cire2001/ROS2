import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool, String
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker, MarkerArray
from exercice4.vec2d import Vec2D
from exercice4.map2d import Map2D
from exercice4.astar import AStar
import time
import math

class SurveillanceRobot(Node):
    def __init__(self, robot_id=1):
        super().__init__(f'surveillance_robot_{robot_id}')

        self.robot_id = robot_id
        self.get_logger().info(f"Starting Surveillance Robot {robot_id}")

        # Position actuelle
        self.current_position = Vec2D(10.0 + 10.0 * robot_id, 10.0)
        self.target_position = None
        self.path = []
        self.status = 'idle'
        self.surveillance_time = 10.0
        self.surveillance_start_time = None

        # Obtenir la carte
        self.cli = self.create_client(GetMap, '/map_server/map')
        self.req = GetMap.Request()
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.map_response = self.future.result()
        self.map = Map2D(self.map_response.map)
        self.astar = AStar(self.map)

        # Publications standard
        self.position_pub = self.create_publisher(
            PoseStamped,
            f'/robot_{robot_id}/current_position',
            10)

        self.status_pub = self.create_publisher(
            String,
            f'/robot_{robot_id}/status',
            10)

        # Publications pour les marqueurs
        self.marker_pub = self.create_publisher(
            Marker,
            f'/robot_{robot_id}/marker',
            10)

        # Créer également une publication pour le marker_array pour le nettoyer
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            f'/robot_{robot_id}/marker_array',
            10)

        # Abonnements
        self.create_subscription(
            Point,
            f'/robot_{robot_id}/target_position',
            self.target_position_callback,
            10)

        self.create_subscription(
            Bool,
            f'/robot_{robot_id}/stop_command',
            self.stop_command_callback,
            10)

        # CRUCIAL: Nettoyer tous les marqueurs existants
        self.cleanup_all_markers()

        # Créer le marqueur du robot une seule fois
        self.robot_marker = self.create_robot_marker()

        # Timers
        self.update_timer = self.create_timer(0.1, self.update_position)
        self.publish_timer = self.create_timer(0.1, self.publish_position)

        # Publier statut initial
        self.publish_status()

    def cleanup_all_markers(self):
        """Nettoyer tous les marqueurs existants de manière plus efficace"""
        # 1. Envoyer DELETEALL sur le topic marker
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        self.marker_pub.publish(delete_marker)

        # 2. Publier un MarkerArray vide pour nettoyer ce topic également
        empty_array = MarkerArray()
        self.marker_array_pub.publish(empty_array)

        # 3. Supprimer explicitement les marqueurs avec des IDs spécifiques
        for ns in ["robot_1_FINAL", "robot_2_FINAL", "robot_3_FINAL"]:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = 123456
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)

        # Attendre que tout soit nettoyé
        time.sleep(0.5)
        self.get_logger().info("Cleaned up all markers")

    def create_robot_marker(self):
        """Créer un marqueur unique et stable pour le robot"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = f"robot_{self.robot_id}_FINAL"  # Namespace très distinctif
        marker.id = self.robot_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = 3.0
        marker.scale.y = 3.0
        marker.scale.z = 1.0

        # Couleur selon le robot
        if self.robot_id == 1:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # Bleu
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0  # Vert
            marker.color.b = 0.0
        marker.color.a = 1.0

        # Pas de durée de vie limitée
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    def target_position_callback(self, msg):
        """Callback pour la position cible"""
        self.target_position = Vec2D(msg.x, msg.y)
        self.get_logger().info(f"New target position: ({msg.x}, {msg.y})")

        # Calculer un nouveau chemin
        self.path = self.astar.find_path(self.current_position, self.target_position)

        if not self.path:
            self.get_logger().warn(f"No path found to target ({msg.x}, {msg.y})")
        else:
            self.get_logger().info(f"Path found with {len(self.path)} steps")
            self.status = 'moving'
            self.publish_status()

    def stop_command_callback(self, msg):
        """Callback pour la commande d'arrêt"""
        if msg.data:
            self.get_logger().info("Stop command received")
            self.path = []
            self.status = 'idle'
            self.publish_status()

    def update_position(self):
        """Mettre à jour la position du robot"""
        if self.status == 'moving' and self.path:
            # Prendre le prochain point du chemin
            next_point = self.path[0]

            # Calculer la direction et la distance
            direction = next_point - self.current_position
            distance = direction.length()

            # Si on est assez proche du prochain point, passer au suivant
            if distance < 0.5:
                self.path.pop(0)

                # Si on a atteint la fin du chemin
                if not self.path and self.target_position:
                    if self.current_position.distance_to(self.target_position) < 1.0:
                        self.get_logger().info("Target reached, starting surveillance")
                        self.status = 'surveilling'
                        self.surveillance_start_time = time.time()
                        self.publish_status()
            else:
                # Déplacer le robot vers le prochain point
                move_speed = 30.0  # Augmenté à 30.0 pour un mouvement plus fluide
                move_distance = min(distance, move_speed * 0.1)  # 0.1 est le delta time

                # Normaliser la direction et multiplier par la distance à parcourir
                normalized_direction = direction.normalized()
                self.current_position = self.current_position + normalized_direction * move_distance

        elif self.status == 'surveilling':
            # Vérifier si le temps de surveillance est écoulé
            if time.time() - self.surveillance_start_time >= self.surveillance_time:
                self.get_logger().info("Surveillance completed")
                self.status = 'idle'
                self.target_position = None
                self.publish_status()

    def publish_position(self):
        """Publier la position actuelle"""
        # Publier la position pour le planning
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.current_position.x
        msg.pose.position.y = self.current_position.y
        msg.pose.position.z = 0.0
        self.position_pub.publish(msg)

        # Mettre à jour et publier le marqueur du robot
        self.robot_marker.header.stamp = self.get_clock().now().to_msg()
        self.robot_marker.pose.position.x = self.current_position.x
        self.robot_marker.pose.position.y = self.current_position.y
        self.robot_marker.pose.position.z = 0.0
        self.robot_marker.pose.orientation.w = 1.0
        self.marker_pub.publish(self.robot_marker)

    def publish_status(self):
        """Publier le statut du robot"""
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {self.status}")

def main(args=None):
    rclpy.init(args=args)

    # Récupérer l'ID du robot depuis les arguments
    import sys
    robot_id = 1

    # Parcourir les arguments pour trouver un argument qui n'est pas une option ROS
    for i, arg in enumerate(sys.argv):
        if not arg.startswith('--'):
            try:
                # Essayer de convertir l'argument en entier
                if i > 0:  # Ignorer le premier argument qui est le nom du script
                    robot_id = int(arg)
                    break
            except ValueError:
                # Si la conversion échoue, ignorer cet argument
                pass

    print(f"Starting robot with ID: {robot_id}")
    node = SurveillanceRobot(robot_id)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
