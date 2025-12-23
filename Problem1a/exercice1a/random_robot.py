import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped, PoseStamped
from exercice1.map2d import Map2D
from exercice1.vec2d import Vec2D
import random

class RandomRobot(Node):
    def __init__(self):
        super().__init__('random_robot')

        # Initialisation de la position du robot à None
        self.robot_position = None
        # Initialisation de la direction du robot à None
        self.direction = None

        # Client pour le service de carte
        self.cli = self.create_client(GetMap, '/map_server/map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()

        # Récupération de la carte
        self.map_response = self.get_map()
        self.map = Map2D(self.map_response.map)

        # Souscription au topic clicked_point
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.receive_point,
            10)
        self.subscription  # prevent unused variable warning

        # Publication sur le topic robot_pos
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'robot_pos',
            10)

        # Timer pour publier la position du robot régulièrement
        timer_period = 0.2  # secondes (augmentation de la fréquence)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Mon noeud est demarre !")

    def get_map(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def receive_point(self, msg):
        # Afficher les coordonnées du point
        self.get_logger().info('Point reçu: x=%f, y=%f, z=%f' %
                              (msg.point.x, msg.point.y, msg.point.z))

        # Convertir en Vec2D
        point = Vec2D(msg.point.x, msg.point.y)

        # Vérifier si le point n'est pas un mur
        if not self.map.is_wall(point):
            # Si ce n'est pas un mur, stocker les coordonnées
            self.robot_position = point
            self.get_logger().info(f'Position du robot mise à jour: {self.robot_position}')

            # Choisir une direction aléatoire initiale
            self.choose_random_direction()
        else:
            self.get_logger().info('Point ignoré: position occupée par un mur')

    def format_pose(self, pos):
        # Créer un message PoseStamped
        msg = PoseStamped()

        # Définir l'en-tête du message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Définir la position
        msg.pose.position.x = pos.x
        msg.pose.position.y = pos.y
        msg.pose.position.z = 0.0

        # Définir l'orientation (quaternion)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0  # Orientation par défaut (pas de rotation)

        return msg

    def choose_random_direction(self):
        # Liste des directions possibles (haut, bas, gauche, droite)
        possible_directions = [
            Vec2D(0.2, 0.0),  # droite
            Vec2D(-0.2, 0.0), # gauche
            Vec2D(0.0, 0.2),  # haut
            Vec2D(0.0, -0.2)  # bas
        ]

        # Mélanger les directions
        random.shuffle(possible_directions)

        # Trouver une direction qui ne mène pas à un mur
        for dir in possible_directions:
            next_pos = Vec2D(self.robot_position.x + dir.x, self.robot_position.y + dir.y)
            if not self.map.is_wall(next_pos):
                self.direction = dir
                self.get_logger().info(f'Nouvelle direction choisie: {self.direction}')
                return

        # Si toutes les directions mènent à un mur, ne pas bouger
        self.direction = Vec2D(0.0, 0.0)
        self.get_logger().info('Aucune direction possible, robot immobile')

    def timer_callback(self):
        # Si la position initiale n'a pas été définie, ne rien faire
        if self.robot_position is None:
            return

        # Si la direction n'a pas été définie, en choisir une
        if self.direction is None:
            self.choose_random_direction()
            return

        # Calculer la nouvelle position
        next_pos = Vec2D(self.robot_position.x + self.direction.x, self.robot_position.y + self.direction.y)

        # Vérifier si la nouvelle position est un mur
        if self.map.is_wall(next_pos):
            # Si c'est un mur, choisir une nouvelle direction
            self.get_logger().info('Mur détecté, changement de direction')
            self.choose_random_direction()
            return

        # Mettre à jour la position du robot
        self.robot_position = next_pos

        # Créer et publier le message
        msg = self.format_pose(self.robot_position)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomRobot()

    # Afficher les dimensions de la carte
    node.get_logger().info(
        "Dimensions de la carte : (%d, %d) " %
        (node.map_response.map.info.width, node.map_response.map.info.height))

    # Tester différentes positions
    positions = [Vec2D(0, 0), Vec2D(79, 79), Vec2D(17, 51), Vec2D(3, 3)]
    for pos in positions:
        if node.map.is_wall(pos):
            node.get_logger().info(f"Position {pos} : occupé")
        else:
            node.get_logger().info(f"Position {pos} : libre")

    # Attendre les messages du topic clicked_point
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
