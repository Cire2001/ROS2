# minimax_planner.py (version améliorée)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Int32, Bool, String
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker, MarkerArray
from exercice4.vec2d import Vec2D
from exercice4.map2d import Map2D
from exercice4.astar import AStar
import time
import math
import numpy as np
import random
from builtin_interfaces.msg import Duration

class MinimaxPlanner(Node):
    def __init__(self):
        super().__init__('minimax_planner')

        # Obtenir la carte
        self.cli = self.create_client(GetMap, '/map_server/map')
        self.req = GetMap.Request()

        # Attendre que le service soit disponible
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Appeler le service
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.map_response = self.future.result()
        self.map = Map2D(self.map_response.map)
        self.astar = AStar(self.map)

        # Générer des points de surveillance aléatoires
        self.surveillance_points = self.generate_random_surveillance_points(6)
        self.get_logger().info(f"Generated {len(self.surveillance_points)} surveillance points")
        for i, point in enumerate(self.surveillance_points):
            self.get_logger().info(f"Point {i}: ({point.x}, {point.y})")

        # Positions des robots
        self.robot1_position = None
        self.robot2_position = None
        self.robot1_target = None
        self.robot2_target = None
        self.robot1_status = 'idle'
        self.robot2_status = 'idle'

        # Garder trace des dernières visites
        self.point_last_visit = {}  # {point_idx: timestamp}

        # Importance relative des points basée sur le temps depuis la dernière visite
        self.point_importance = {i: 1.0 for i in range(len(self.surveillance_points))}

        # Temps minimal entre deux visites du même point
        self.min_revisit_time = 20.0  # secondes

        # Publisher pour les marqueurs des points de surveillance
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/surveillance_points',
            10)

        # Abonnements aux positions des robots
        self.create_subscription(
            PoseStamped,
            '/robot_1/current_position',
            self.robot1_position_callback,
            10)

        self.create_subscription(
            PoseStamped,
            '/robot_2/current_position',
            self.robot2_position_callback,
            10)

        self.create_subscription(
            String,
            '/robot_1/status',
            self.robot1_status_callback,
            10)

        self.create_subscription(
            String,
            '/robot_2/status',
            self.robot2_status_callback,
            10)

        # Publications pour les cibles des robots
        self.target1_pub = self.create_publisher(
            Point,
            '/robot_1/target_position',
            10)

        self.target2_pub = self.create_publisher(
            Point,
            '/robot_2/target_position',
            10)

        self.stop1_pub = self.create_publisher(
            Bool,
            '/robot_1/stop_command',
            10)

        self.stop2_pub = self.create_publisher(
            Bool,
            '/robot_2/stop_command',
            10)

        # Timer pour planifier la surveillance
        self.planning_timer = self.create_timer(2.0, self.plan_surveillance)

        # Timer pour mettre à jour l'importance des points
        self.importance_timer = self.create_timer(5.0, self.update_point_importance)

        # Timer pour publier les marqueurs
        self.marker_timer = self.create_timer(1.0, self.publish_surveillance_points)

        self.get_logger().info('Minimax Planner started for both robots')

        # Publier les points de surveillance immédiatement
        self.publish_surveillance_points()

    def generate_random_surveillance_points(self, num_points):
        """Générer des points de surveillance aléatoires"""
        points = []
        max_attempts = 50  # Nombre maximum de tentatives pour trouver un point valide

        # Dimensions de la carte
        width = self.map.map.info.width * self.map.map.info.resolution
        height = self.map.map.info.height * self.map.map.info.resolution
        origin_x = self.map.map.info.origin.position.x
        origin_y = self.map.map.info.origin.position.y

        # Générer des points aléatoires dans la carte
        for _ in range(num_points):
            for attempt in range(max_attempts):
                # Générer des coordonnées aléatoires dans les limites de la carte
                x = random.uniform(origin_x + 5.0, origin_x + width - 5.0)
                y = random.uniform(origin_y + 5.0, origin_y + height - 5.0)
                point = Vec2D(x, y)

                # Vérifier que le point n'est pas dans un mur
                if not self.map.is_wall(point):
                    # Vérifier que le point est suffisamment éloigné des autres points
                    too_close = False
                    for existing_point in points:
                        if point.distance_to(existing_point) < 10.0:  # Distance minimale entre les points
                            too_close = True
                            break

                    if not too_close:
                        points.append(point)
                        break

            # Si nous n'avons pas trouvé de point valide après max_attempts, utiliser un point par défaut
            if len(points) <= _:
                default_point = Vec2D(45.0, 45.0)
                if not self.map.is_wall(default_point):
                    points.append(default_point)
                else:
                    # Chercher un point non-mur proche
                    for dx in range(-10, 11, 2):
                        for dy in range(-10, 11, 2):
                            test_point = Vec2D(45.0 + dx, 45.0 + dy)
                            if not self.map.is_wall(test_point):
                                points.append(test_point)
                                break
                        if len(points) > _:
                            break

        return points

    def publish_surveillance_points(self):
        """Publier les marqueurs pour les points de surveillance"""
        marker_array = MarkerArray()

        for i, point in enumerate(self.surveillance_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "surveillance_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = 0.0
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0

            # Couleur rouge de base
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            # Points actuellement ciblés sont légèrement plus gros
            if i == self.robot1_target or i == self.robot2_target:
                marker.scale.x = 2.5
                marker.scale.y = 2.5
                marker.scale.z = 2.5

            # Ajouter une durée de vie au marqueur
            lifetime = Duration()
            lifetime.sec = 2
            lifetime.nanosec = 0
            marker.lifetime = lifetime

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def robot1_position_callback(self, msg):
        """Callback pour la position du robot 1"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot1_position = Vec2D(x, y)

    def robot2_position_callback(self, msg):
        """Callback pour la position du robot 2"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot2_position = Vec2D(x, y)

    def robot1_status_callback(self, msg):
        """Callback pour le statut du robot 1"""
        old_status = self.robot1_status
        self.robot1_status = msg.data

        # Si le robot vient de terminer une surveillance
        if old_status == 'surveilling' and self.robot1_status == 'idle' and self.robot1_target is not None:
            # Mettre à jour le timestamp de dernière visite
            self.point_last_visit[self.robot1_target] = time.time()
            self.get_logger().info(f"Robot 1 completed surveillance of point {self.robot1_target}")
            self.robot1_target = None

            # Déclencher immédiatement une nouvelle planification
            self.plan_surveillance()

    def robot2_status_callback(self, msg):
        """Callback pour le statut du robot 2"""
        old_status = self.robot2_status
        self.robot2_status = msg.data

        # Si le robot vient de terminer une surveillance
        if old_status == 'surveilling' and self.robot2_status == 'idle' and self.robot2_target is not None:
            # Mettre à jour le timestamp de dernière visite
            self.point_last_visit[self.robot2_target] = time.time()
            self.get_logger().info(f"Robot 2 completed surveillance of point {self.robot2_target}")
            self.robot2_target = None

            # Déclencher immédiatement une nouvelle planification
            self.plan_surveillance()

    def update_point_importance(self):
        """Mettre à jour l'importance de chaque point basée sur le temps depuis la dernière visite"""
        current_time = time.time()

        for i in range(len(self.surveillance_points)):
            if i in self.point_last_visit:
                time_since_visit = current_time - self.point_last_visit[i]
                # L'importance augmente avec le temps écoulé depuis la dernière visite
                self.point_importance[i] = max(1.0, time_since_visit / 10.0)
            else:
                # Les points jamais visités ont une importance maximale
                self.point_importance[i] = 5.0

    def calculate_utility(self, robot_position, point_idx):
        """Calculer l'utilité d'un point pour un robot"""
        point = self.surveillance_points[point_idx]
        path = self.astar.find_path(robot_position, point)

        if not path:
            return float('-inf')  # Pas de chemin trouvé

        # L'utilité est basée sur l'importance du point et la distance
        distance_utility = 1000.0 / (len(path) + 1)
        importance = self.point_importance[point_idx]

        return distance_utility * importance

    def minimax_decision(self, robot1_available, robot2_available):
        """Prendre une décision basée sur l'algorithme minimax"""
        # Vérifier si on a les positions des robots
        if not self.robot1_position or not self.robot2_position:
            return None, None

        # Déterminer les points candidats
        current_time = time.time()
        candidate_points = []

        for i in range(len(self.surveillance_points)):
            # Vérifier si le point a été visité récemment
            if i in self.point_last_visit:
                time_since_visit = current_time - self.point_last_visit[i]
                if time_since_visit < self.min_revisit_time:
                    continue  # Trop récent, on ignore ce point

            # Vérifier si le point est déjà ciblé
            if (i == self.robot1_target and not robot1_available) or (i == self.robot2_target and not robot2_available):
                continue

            candidate_points.append(i)

        if not candidate_points:
            return None, None

        # Matrice de gain pour le jeu
        payoff_matrix = {}

        # Calculer les utilités pour toutes les combinaisons de points
        for i in candidate_points:
            payoff_matrix[i] = {}
            for j in candidate_points:
                if i == j:  # Les robots ne peuvent pas aller au même point
                    payoff_matrix[i][j] = (float('-inf'), float('-inf'))
                else:
                    utility_i = self.calculate_utility(self.robot1_position, i)
                    utility_j = self.calculate_utility(self.robot2_position, j)
                    payoff_matrix[i][j] = (utility_i, utility_j)

        # Appliquer l'algorithme minimax
        best_i = None
        best_j = None
        best_value = float('-inf')

        for i in candidate_points:
            # Robot 1 maximise son utilité
            min_value_i = float('inf')
            best_response_j = None

            for j in candidate_points:
                if i == j:
                    continue

                # Robot 2 maximise son utilité
                value_j = payoff_matrix[i][j][1]

                # Trouver la meilleure réponse du robot 2
                if value_j > best_value:
                    best_response_j = j

            # Si une meilleure réponse est trouvée
            if best_response_j is not None:
                value_i = payoff_matrix[i][best_response_j][0]

                # Robot 1 choisit la stratégie qui maximise son utilité
                if value_i > best_value:
                    best_value = value_i
                    best_i = i
                    best_j = best_response_j

        # Si aucune solution n'est trouvée, utiliser une approche plus simple
        if best_i is None or best_j is None:
            # Trouver les meilleurs points individuellement
            best_i = max(candidate_points, key=lambda i: self.calculate_utility(self.robot1_position, i), default=None)

            # Exclure le point choisi par le robot 1
            remaining_points = [j for j in candidate_points if j != best_i]
            if remaining_points:
                best_j = max(remaining_points, key=lambda j: self.calculate_utility(self.robot2_position, j), default=None)

        return best_i, best_j

    def plan_surveillance(self):
        """Planifier la surveillance avec l'algorithme minimax"""
        # Vérifier si on a les positions des robots
        if not self.robot1_position or not self.robot2_position:
            self.get_logger().warn("Waiting for robot positions")
            return

        # Vérifier si les robots sont disponibles
        robot1_available = self.robot1_status == 'idle' and self.robot1_target is None
        robot2_available = self.robot2_status == 'idle' and self.robot2_target is None

        # Si aucun robot n'est disponible, ne rien faire
        if not robot1_available and not robot2_available:
            return

        # Appliquer l'algorithme minimax
        point1_idx, point2_idx = self.minimax_decision(robot1_available, robot2_available)

        # Assigner les points aux robots disponibles
        if robot1_available and point1_idx is not None:
            target1 = self.surveillance_points[point1_idx]
            target_msg = Point()
            target_msg.x = float(target1.x)
            target_msg.y = float(target1.y)
            target_msg.z = 0.0
            self.target1_pub.publish(target_msg)
            self.robot1_target = point1_idx
            self.get_logger().info(f"Robot 1 assigned to point {point1_idx}: ({target1.x}, {target1.y})")

        if robot2_available and point2_idx is not None:
            target2 = self.surveillance_points[point2_idx]
            target_msg = Point()
            target_msg.x = float(target2.x)
            target_msg.y = float(target2.y)
            target_msg.z = 0.0
            self.target2_pub.publish(target_msg)
            self.robot2_target = point2_idx
            self.get_logger().info(f"Robot 2 assigned to point {point2_idx}: ({target2.x}, {target2.y})")

def main(args=None):
    rclpy.init(args=args)
    node = MinimaxPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
