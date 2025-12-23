# joint_planner.py (version corrigée suivant l'algorithme du cours)
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

class JointPlanner(Node):
    def __init__(self):
        super().__init__('joint_planner')

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

        self.get_logger().info('Joint Planner started for both robots')

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

    def plan_surveillance(self):
        """Planifier la surveillance avec l'algorithme de planification jointe"""
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

        # Implementer l'algorithme de planification jointe
        # Déterminer les points candidats pour chaque robot
        current_time = time.time()
        candidate_points = []

        for i in range(len(self.surveillance_points)):
            # Vérifier si le point a été visité récemment
            if i in self.point_last_visit:
                time_since_visit = current_time - self.point_last_visit[i]
                if time_since_visit < self.min_revisit_time:
                    continue  # Trop récent, on ignore ce point

            # Vérifier si le point est déjà ciblé par un robot
            if i == self.robot1_target or i == self.robot2_target:
                continue

            candidate_points.append(i)

        # Si aucun point n'est disponible, ne rien faire
        if not candidate_points:
            return

        # Si un seul robot est disponible, lui assigner le meilleur point
        if robot1_available and not robot2_available:
            best_point = self.find_best_point_for_robot(1, candidate_points)
            if best_point is not None:
                self.assign_point_to_robot(1, best_point)
            return

        if robot2_available and not robot1_available:
            best_point = self.find_best_point_for_robot(2, candidate_points)
            if best_point is not None:
                self.assign_point_to_robot(2, best_point)
            return

        # Si les deux robots sont disponibles, trouver la meilleure combinaison
        best_utility = float('-inf')
        best_allocation = None

        # Évaluer toutes les combinaisons possibles de points
        for i in candidate_points:
            for j in candidate_points:
                if i == j:
                    continue  # Les robots ne doivent pas aller au même point

                # Calculer l'utilité de cette allocation
                utility_i = self.calculate_utility(self.robot1_position, self.surveillance_points[i], i)
                utility_j = self.calculate_utility(self.robot2_position, self.surveillance_points[j], j)
                joint_utility = utility_i + utility_j

                if joint_utility > best_utility:
                    best_utility = joint_utility
                    best_allocation = (i, j)

        # Assigner les points aux robots
        if best_allocation:
            if robot1_available:
                self.assign_point_to_robot(1, best_allocation[0])
            if robot2_available:
                self.assign_point_to_robot(2, best_allocation[1])

    def calculate_utility(self, robot_position, point, point_idx):
        """Calculer l'utilité d'assigner un point à un robot"""
        path = self.astar.find_path(robot_position, point)
        if not path:
            return float('-inf')  # Pas de chemin trouvé

        # L'utilité est basée sur l'importance du point et la distance
        distance_utility = 1000.0 / (len(path) + 1)
        importance = self.point_importance[point_idx]

        return distance_utility * importance

    def find_best_point_for_robot(self, robot_id, candidate_points):
        """Trouver le meilleur point pour un robot spécifique"""
        best_point = None
        best_utility = float('-inf')

        robot_position = self.robot1_position if robot_id == 1 else self.robot2_position

        for point_idx in candidate_points:
            point = self.surveillance_points[point_idx]
            utility = self.calculate_utility(robot_position, point, point_idx)

            if utility > best_utility:
                best_utility = utility
                best_point = point_idx

        return best_point

    def assign_point_to_robot(self, robot_id, point_idx):
        """Assigner un point à un robot"""
        target = self.surveillance_points[point_idx]
        target_msg = Point()
        target_msg.x = float(target.x)
        target_msg.y = float(target.y)
        target_msg.z = 0.0

        if robot_id == 1:
            self.target1_pub.publish(target_msg)
            self.robot1_target = point_idx
            self.get_logger().info(f"Robot 1 assigned to point {point_idx}: ({target.x}, {target.y})")
        else:
            self.target2_pub.publish(target_msg)
            self.robot2_target = point_idx
            self.get_logger().info(f"Robot 2 assigned to point {point_idx}: ({target.x}, {target.y})")

def main(args=None):
    rclpy.init(args=args)
    node = JointPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
