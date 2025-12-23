# destination_manager.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from exercice1.vec2d import Vec2D
import math

class DestinationManager(Node):
    def __init__(self):
        super().__init__('destination_manager')

        # Destination commune
        self.destination = None

        # Liste des positions occupées
        self.occupied_positions = []

        # Nombre de robots
        self.num_robots = 3  # À ajuster selon votre configuration

        # Souscriptions
        for i in range(1, self.num_robots + 1):
            self.create_subscription(
                PointStamped,
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
        # Mettre à jour la position du robot dans la liste des positions occupées
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

    def generate_positions_around(self, center, count):
        positions = []

        # Rayon autour de la destination
        radius = 2.0

        # Générer des positions en cercle autour de la destination
        for i in range(count):
            angle = 2 * math.pi * i / count
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)
            positions.append(Vec2D(x, y))

        return positions

def main(args=None):
    rclpy.init(args=args)
    node = DestinationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
