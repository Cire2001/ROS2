# destination_manager.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import String
from exercice1.vec2d import Vec2D
from nav_msgs.srv import GetMap
from exercice1.map2d import Map2D
import math
import time

class DestinationManager(Node):
    def __init__(self):
        super().__init__('destination_manager')

        # Destination commune
        self.destination = None

        # Liste des positions occupées
        self.occupied_positions = []

        # Nombre de robots
        self.num_robots = 3  # À ajuster selon votre configuration

        # Récupération de la carte pour les vérifications
        self.map = None
        self.cli = self.create_client(GetMap, '/map_server/map')

        # Attendre le service avec un timeout
        service_available = False
        timeout_time = time.time() + 10.0  # 10 secondes de timeout

        while not service_available and time.time() < timeout_time:
            service_available = self.cli.wait_for_service(timeout_sec=1.0)
            if not service_available:
                self.get_logger().info('Map service not available, waiting again...')

        if service_available:
            self.req = GetMap.Request()
            self.map_response = self.get_map()
            self.map = Map2D(self.map_response.map)
            self.get_logger().info('Map service connected successfully')
        else:
            self.get_logger().warn('Map service not available after timeout. Position verification will be limited.')

        # Souscriptions - utilise PoseStamped au lieu de PointStamped
        for i in range(1, self.num_robots + 1):
            self.create_subscription(
                PoseStamped,  # Changé de PointStamped à PoseStamped
                f'/robot_{i}/current_position',
                lambda msg, robot_id=i: self.robot_position_callback(msg, robot_id),
                10)

        # Souscription pour définir la destination
        self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)

        self.create_subscription(
            Point,
            '/set_destination',
            self.set_destination_callback,
            10)

        # Publications pour envoyer les cibles aux robots
        self.target_pubs = {}
        for i in range(1, self.num_robots + 1):
            self.target_pubs[i] = self.create_publisher(
                Point,
                f'/robot_{i}/target_position',
                10)

        self.get_logger().info('Destination Manager started')

    def get_map(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def clicked_point_callback(self, msg):
        # Définir la destination en cliquant sur la carte
        self.destination = Vec2D(msg.point.x, msg.point.y)
        self.get_logger().info(f'Destination set to: ({self.destination.x}, {self.destination.y})')
        self.assign_positions()

    def set_destination_callback(self, msg):
        # Définir la destination via un message
        self.destination = Vec2D(msg.x, msg.y)
        self.get_logger().info(f'Destination set to: ({self.destination.x}, {self.destination.y})')
        self.assign_positions()

    def robot_position_callback(self, msg, robot_id):
        # Adapté pour traiter des messages PoseStamped
        position = Vec2D(msg.pose.position.x, msg.pose.position.y)

        # Mettre à jour ou ajouter la position dans la liste
        updated = False
        for i, (rid, pos) in enumerate(self.occupied_positions):
            if rid == robot_id:
                self.occupied_positions[i] = (robot_id, position)
                updated = True
                break

        if not updated:
            self.occupied_positions.append((robot_id, position))

    def assign_positions(self):
        if self.destination is None:
            return

        # Générer des positions autour de la destination
        positions = self.generate_positions_around(self.destination, self.num_robots)

        # Assigner les positions aux robots
        for i, position in enumerate(positions):
            robot_id = i + 1
            target_msg = Point()
            target_msg.x = position.x
            target_msg.y = position.y
            target_msg.z = 0.0

            self.target_pubs[robot_id].publish(target_msg)
            self.get_logger().info(f'Assigned position ({position.x}, {position.y}) to robot {robot_id}')

    def is_wall_position(self, pos):
        # Vérifier si une position est dans un mur
        if self.map is None:
            return False  # Sans carte, on suppose que la position est accessible

        return self.map.is_wall(pos)

    def generate_positions_around(self, center, count):
        positions = []

        # Rayon autour de la destination
        radius = 2.0

        # Générer des positions en cercle autour de la destination
        attempts = 0
        max_attempts = 20  # Limiter le nombre de tentatives

        while len(positions) < count and attempts < max_attempts:
            for i in range(count):
                angle = 2 * math.pi * i / count
                x = center.x + radius * math.cos(angle)
                y = center.y + radius * math.sin(angle)
                pos = Vec2D(x, y)

                # Vérifier si la position est accessible (pas un mur)
                if not self.is_wall_position(pos):
                    # Correction ici: utiliser distance_to au lieu de distance
                    if not any(existing_pos.distance_to(pos) < 0.5 for existing_pos in positions):
                        positions.append(pos)
                        self.get_logger().info(f'Valid position found: ({pos.x}, {pos.y})')

            # Si nous n'avons pas assez de positions, augmenter le rayon et réessayer
            if len(positions) < count:
                radius += 0.5
                attempts += 1
                self.get_logger().info(f'Increasing radius to {radius}, attempt {attempts}')

        # Si nous n'avons toujours pas assez de positions, utiliser celles que nous avons
        return positions[:count]

def main(args=None):
    rclpy.init(args=args)
    node = DestinationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
