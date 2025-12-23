# rendezvous_manager.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Bool
from exercice2.vec2d import Vec2D
import math

class RendezvousManager(Node):
    def __init__(self):
        super().__init__('rendezvous_manager')

        # Positions initiales des robots
        self.initial_positions = {}
        # Positions actuelles des robots
        self.robot_positions = {}
        # Cibles des robots
        self.robot_targets = {}

        # État de rendez-vous
        self.rendezvous_started = False
        self.robots_stopped = set()  # Ensemble des robots qui se sont arrêtés

        # Abonnements aux positions des robots
        self.create_subscription(
            PoseStamped,
            '/robot_1/current_position',
            lambda msg: self.robot_position_callback(msg, 1),
            10)

        self.create_subscription(
            PoseStamped,
            '/robot_2/current_position',
            lambda msg: self.robot_position_callback(msg, 2),
            10)

        # Abonnement au déclencheur de rendez-vous
        self.create_subscription(
            String,
            '/trigger_rendezvous',
            self.trigger_callback,
            10)

        # Publications pour les cibles des robots
        self.robot1_target_pub = self.create_publisher(
            Point,
            '/robot_1/target_position',
            10)

        self.robot2_target_pub = self.create_publisher(
            Point,
            '/robot_2/target_position',
            10)

        # Publications pour les commandes d'arrêt
        self.robot1_stop_pub = self.create_publisher(
            Bool,
            '/robot_1/stop_command',
            10)

        self.robot2_stop_pub = self.create_publisher(
            Bool,
            '/robot_2/stop_command',
            10)

        # Timer pour vérifier la progression des robots
        self.check_timer = self.create_timer(0.1, self.check_progress)

        self.get_logger().info('Rendezvous Manager started')

    def robot_position_callback(self, msg, robot_id):
        # Extraire la position du message
        x = msg.pose.position.x
        y = msg.pose.position.y
        position = Vec2D(x, y)

        # Si c'est la première fois qu'on reçoit cette position, la considérer comme initiale
        if robot_id not in self.initial_positions and not self.rendezvous_started:
            self.initial_positions[robot_id] = position
            self.get_logger().info(f'Initial position for robot {robot_id}: ({x}, {y})')

        # Enregistrer la position actuelle
        self.robot_positions[robot_id] = position

    def trigger_callback(self, msg):
        self.get_logger().info('Rendezvous triggered')

        # Vérifier si nous avons assez de positions
        if len(self.robot_positions) < 2:
            self.get_logger().warn('Not enough robot positions to calculate rendezvous point')
            self.get_logger().info(f'Current robot positions: {self.robot_positions}')
            return

        # Réinitialiser l'état
        self.rendezvous_started = True
        self.robots_stopped.clear()

        # Enregistrer les cibles
        self.robot_targets[1] = self.robot_positions[2]  # Robot 1 -> Robot 2
        self.robot_targets[2] = self.robot_positions[1]  # Robot 2 -> Robot 1

        # Envoyer à chaque robot la position de l'autre robot comme cible
        self.send_target_to_robot(1, self.robot_positions[2])  # Robot 1 va vers Robot 2
        self.send_target_to_robot(2, self.robot_positions[1])  # Robot 2 va vers Robot 1

        # Afficher les positions et cibles pour débogage
        self.get_logger().warn(f'Robot 1 at {self.robot_positions[1].x}, {self.robot_positions[1].y} -> going to {self.robot_positions[2].x}, {self.robot_positions[2].y}')
        self.get_logger().warn(f'Robot 2 at {self.robot_positions[2].x}, {self.robot_positions[2].y} -> going to {self.robot_positions[1].x}, {self.robot_positions[1].y}')

    def check_progress(self):
        if not self.rendezvous_started or len(self.robots_stopped) == 2:
            return  # Pas de rendez-vous en cours ou déjà terminé

        # Vérifier si chaque robot a atteint sa cible
        for robot_id in [1, 2]:
            if robot_id in self.robots_stopped:
                continue  # Ce robot s'est déjà arrêté

            if robot_id not in self.robot_positions or robot_id not in self.robot_targets:
                continue  # Pas assez d'informations pour ce robot

            # Position actuelle et cible
            current_pos = self.robot_positions[robot_id]
            target_pos = self.robot_targets[robot_id]

            # Si le robot a croisé l'autre robot (ils sont proches)
            # Calculer la distance entre les robots
            other_robot_id = 2 if robot_id == 1 else 1
            if other_robot_id not in self.robot_positions:
                continue

            other_pos = self.robot_positions[other_robot_id]
            distance = math.sqrt((current_pos.x - other_pos.x)**2 + (current_pos.y - other_pos.y)**2)

            # Si les robots sont proches l'un de l'autre, les arrêter
            if distance < 3.0:
                self.get_logger().warn(f'Robots are close! Distance: {distance:.2f}')
                self.get_logger().warn(f'Robot {robot_id} position: ({current_pos.x:.1f}, {current_pos.y:.1f})')
                self.get_logger().warn(f'Robot {other_robot_id} position: ({other_pos.x:.1f}, {other_pos.y:.1f})')

                # Arrêter les deux robots immédiatement
                self.stop_both_robots()
                return

            # Arrêter aussi si un robot a presque atteint sa cible
            distance_to_target = math.sqrt((current_pos.x - target_pos.x)**2 + (current_pos.y - target_pos.y)**2)
            if distance_to_target < 3.0:
                self.get_logger().warn(f'Robot {robot_id} has reached its target! Distance: {distance_to_target:.2f}')
                self.stop_both_robots()
                return

    def stop_both_robots(self):
        # Marquer les deux robots comme arrêtés
        self.robots_stopped = {1, 2}

        # Publier la commande d'arrêt pour les deux robots
        stop_msg = Bool()
        stop_msg.data = True

        # Publier 5 fois pour être sûr
        for _ in range(5):
            self.robot1_stop_pub.publish(stop_msg)
            self.robot2_stop_pub.publish(stop_msg)

        self.get_logger().warn('BOTH ROBOTS ORDERED TO STOP!')

        # Publier à nouveau leurs positions actuelles comme cibles
        # pour qu'ils restent à cet endroit
        if 1 in self.robot_positions:
            point_msg = Point()
            point_msg.x = float(self.robot_positions[1].x)
            point_msg.y = float(self.robot_positions[1].y)
            self.robot1_target_pub.publish(point_msg)

        if 2 in self.robot_positions:
            point_msg = Point()
            point_msg.x = float(self.robot_positions[2].x)
            point_msg.y = float(self.robot_positions[2].y)
            self.robot2_target_pub.publish(point_msg)

    def send_target_to_robot(self, robot_id, target_position):
        # Créer le message Point
        point_msg = Point()
        point_msg.x = float(target_position.x)
        point_msg.y = float(target_position.y)
        point_msg.z = 0.0

        # Publier sur le topic du robot approprié
        if robot_id == 1:
            self.robot1_target_pub.publish(point_msg)
        elif robot_id == 2:
            self.robot2_target_pub.publish(point_msg)

        self.get_logger().info(f'Sent target position to robot {robot_id}: ({point_msg.x}, {point_msg.y})')

def main(args=None):
    rclpy.init(args=args)
    node = RendezvousManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
