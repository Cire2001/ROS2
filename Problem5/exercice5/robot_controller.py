#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
from exercice5.map2d import Map2D
from exercice5.vec2d import Vec2D
from exercice5.astar import AStar
from exercice5.stochastic_environment import StochasticEnvironment
import time
import math
import re

class RobotController(Node):
    def __init__(self, robot_id, x=None, y=None):
        super().__init__(f'robot_{robot_id}')
        self.robot_id = robot_id
        self.position = None
        self.target = None
        self.path = []
        self.stopped = False
        self.path_timer = None
        self.following_robot_id = -1
        self.is_leader = False
        self.other_robot_positions = {}
        self.other_robot_orientations = {}
        self.last_path_calculation = 0
        self.movement_in_progress = False
        self.orientation = 0.0
        self.final_destination = None
        self.follow_distance = 10.0
        self.jesp_policy = {}
        self.best_response_policy = {}
        self.current_policy = {}  # Politique actuellement utilisée
        self.use_policy = True
        self.num_robots = 3

        # Attributs pour gérer l'arrivée
        self.has_reached_destination = False  # Indique si le robot a atteint sa destination
        self.arrived_robots = set()  # Ensemble des robots arrivés

        # Positions initiales
        if x is not None and y is not None:
            self.position = Vec2D(x, y)
        elif robot_id == 1:
            self.position = Vec2D(20.0, 20.0)
        elif robot_id == 2:
            self.position = Vec2D(35.0, 35.0)
        elif robot_id == 3:
            self.position = Vec2D(60.0, 20.0)

        # Publisher de position
        self.position_pub = self.create_publisher(
            PoseStamped,
            f'/robot_{robot_id}/current_position',
            10)

        # Publisher pour visualiser le chemin du robot
        self.path_marker_pub = self.create_publisher(
            Marker,
            f'/robot_{robot_id}/path_marker',
            10)

        # Publisher d'arrivée
        self.arrival_publisher = self.create_publisher(
            Int32, f'/robot_{robot_id}/arrived', 10
        )

        # Subscription à une commande de cible individuelle
        self.target_sub = self.create_subscription(
            Point,
            f'/robot_{robot_id}/target_position',
            self.target_callback,
            10)

        # Écouter la destination globale comme une target
        self.create_subscription(
            Point,
            '/set_destination',
            self.target_callback,
            10)

        # Souscription au stop
        self.stop_sub = self.create_subscription(
            Bool,
            f'/robot_{robot_id}/stop_command',
            self.stop_callback,
            10)

        # Souscription au robot à suivre
        self.follow_robot_sub = self.create_subscription(
            Int32,
            f'/robot_{robot_id}/follow_robot',
            self.follow_robot_callback,
            10)

        # Souscriptions aux positions des autres robots
        for i in range(1, self.num_robots + 1):
            if i != robot_id:
                self.create_subscription(
                    PoseStamped,
                    f'/robot_{i}/current_position',
                    lambda msg, other_id=i: self.other_robot_position_callback(msg, other_id),
                    10)

                # S'abonner aux arrivées des autres robots
                self.create_subscription(
                    Int32,
                    f'/robot_{i}/arrived',
                    lambda msg, other_id=i: self.robot_arrived_callback(other_id),
                    10
                )

        # Souscriptions aux politiques
        self.create_subscription(
            String,
            '/jesp_policy',
            self.jesp_policy_callback,
            10)

        self.create_subscription(
            String,
            '/best_response_policy',
            self.best_response_policy_callback,
            10)

        # Timers
        self.position_timer = self.create_timer(0.2, self.publish_position)
        self.follow_timer = self.create_timer(1.0, self.update_follow_target)

        # Service GetMap
        self.cli = self.create_client(GetMap, '/map_server/map')
        self.req = GetMap.Request()
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map service not available, waiting...')
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.map_response = self.future.result()
        self.map = Map2D(self.map_response.map)
        self.astar = AStar(self.map)

        # Environnement stochastique pour l'utilisation des politiques
        self.env = StochasticEnvironment(self.map)

        # Publication initiale
        self.publish_position()
        self.get_logger().info(f'Robot {robot_id} initialized at ({self.position.x}, {self.position.y})')

    # ----
    # CALLBACKS
    # ----

    def other_robot_position_callback(self, msg, other_id):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.other_robot_positions[other_id] = Vec2D(x, y)
        # Orientation (simplifiée)
        qx, qy, qz, qw = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.other_robot_orientations[other_id] = math.atan2(siny, cosy)

    def robot_arrived_callback(self, robot_id):
        """Appelé quand un robot est arrivé à destination"""
        self.arrived_robots.add(robot_id)
        self.get_logger().info(f"Robot {robot_id} has arrived at destination")

        # Si le robot qu'on suit est arrivé, on doit s'arrêter derrière lui
        if robot_id == self.following_robot_id and not self.has_reached_destination:
            self.get_logger().info(f"Robot {self.robot_id} will position behind Robot {robot_id}")
            self.position_behind_leader()

    def follow_robot_callback(self, msg):
        robot_to_follow = msg.data
        self.following_robot_id = robot_to_follow
        self.is_leader = (robot_to_follow == -1)
        if robot_to_follow > 0:
            self.get_logger().info(f'Robot {self.robot_id} will follow robot {robot_to_follow}')
        else:
            self.get_logger().info(f'Robot {self.robot_id} is leader or no follow target')

    def jesp_policy_callback(self, msg):
        self.get_logger().info("Received JESP policy")
        self.parse_policy(msg.data, is_jesp=True)

    def best_response_policy_callback(self, msg):
        self.get_logger().info("Received Best Response policy")
        self.parse_policy(msg.data, is_jesp=False)

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info(f'Robot {self.robot_id} received STOP')
            self.stopped = True
            if self.path_timer:
                self.path_timer.cancel()
                self.path_timer = None
            self.path = []
            self.target = None
            self.movement_in_progress = False
            self.publish_position()
        else:
            self.stopped = False
            self.get_logger().info(f'Robot {self.robot_id} resumed')

    def target_callback(self, msg):
        """Réception d'une cible globale ou individuelle → lancement du pathfinding"""
        # CHANGEMENT : on stocke la destination finale
        self.final_destination = Vec2D(msg.x, msg.y)
        self.get_logger().info(f'Final destination set to: ({msg.x}, {msg.y})')

        # Ignorer si arrêté
        if self.stopped:
            self.get_logger().info(f'Robot {self.robot_id} is stopped – ignoring target')
            return

        new_target = Vec2D(msg.x, msg.y)
        # Si cible inchangée, on skip
        if self.target and new_target.distance_to(self.target) < 1.0:
            return

        self.target = new_target
        self.get_logger().info(f'Robot {self.robot_id} received new target: ({new_target.x}, {new_target.y})')

        # Si c'est le leader et qu'une politique est disponible, l'utiliser en priorité
        if self.is_leader and self.use_policy:
            # Toujours essayer d'utiliser une politique d'abord
            if self.jesp_policy or self.best_response_policy:
                self.current_policy = self.jesp_policy if len(self.jesp_policy) > 0 else self.best_response_policy
                self.get_logger().info(f"Leader using policy with {len(self.current_policy)} states")
                self.use_policy_for_movement()
                return

        # Sinon, utiliser A*
        # Protection anti-glitch
        if time.time() - self.last_path_calculation < 3.0:
            self.get_logger().info('Delaying path calculation to avoid glitches')
            self._delay_timer = self.create_timer(3.0, self._delayed_recalculate_path)
        else:
            self.recalculate_path()

    # ----
    # PATHFINDING & MOVEMENT
    # ----
    def _delayed_recalculate_path(self):
        self.destroy_timer(self._delay_timer)
        self.recalculate_path()

    def recalculate_path(self):
        if not self.target or self.stopped:
            return
        self.last_path_calculation = time.time()
        self.path = self.astar.find_path(self.position, self.target)
        if self.path:
            self.get_logger().info(f'Path calculated ({len(self.path)} steps)')
            # Visualiser le chemin
            self.publish_path_marker()

            if self.path_timer:
                self.path_timer.cancel()
            self._start_timer = self.create_timer(1.0, self._delayed_start_movement)
        else:
            self.get_logger().warn('No path found')
            self.movement_in_progress = False

    def _delayed_start_movement(self):
        self.destroy_timer(self._start_timer)
        self.start_movement()

    def start_movement(self):
        if self.stopped or not self.path:
            return
        self.movement_in_progress = True
        # Si leader, démarrer immédiatement
        period = 0.3
        self.path_timer = self.create_timer(period, self.follow_path_callback)
        self.get_logger().info(f'Robot {self.robot_id} started moving ({"leader" if self.is_leader else "follower"})')

    def follow_path_callback(self):
        if self.stopped:
            if self.path_timer:
                self.path_timer.cancel()
            self.movement_in_progress = False
            return

        # Vérifier si nous sommes arrivés à destination
        if self.is_leader and self.final_destination and self.position.distance_to(self.final_destination) < 2.0:
            if not self.has_reached_destination:
                self.get_logger().info(f'Leader Robot {self.robot_id} reached final destination')
                self.has_reached_destination = True
                # Notifier les autres robots que nous sommes arrivés
                arrival_msg = Int32()
                arrival_msg.data = self.robot_id
                self.arrival_publisher.publish(arrival_msg)

            if self.path_timer:
                self.path_timer.cancel()
            self.movement_in_progress = False
            return

        # Pour les suiveurs, vérifier s'ils ont atteint le point derrière leur leader
        if not self.is_leader and self.following_robot_id in self.other_robot_positions:
            leader_pos = self.other_robot_positions[self.following_robot_id]

            # Si le leader est arrivé et que nous sommes proches de notre position finale
            if self.following_robot_id in self.arrived_robots and self.position.distance_to(leader_pos) < self.follow_distance * 1.5:
                if not self.has_reached_destination:
                    self.get_logger().info(f'Follower Robot {self.robot_id} positioned behind Robot {self.following_robot_id}')
                    self.has_reached_destination = True
                    # Notifier les autres robots que nous sommes arrivés
                    arrival_msg = Int32()
                    arrival_msg.data = self.robot_id
                    self.arrival_publisher.publish(arrival_msg)

                # Calculer la position exacte derrière le leader
                self.position_behind_leader()

                if self.path_timer:
                    self.path_timer.cancel()
                self.movement_in_progress = False
                return

        # Si nous n'avons plus de chemin à suivre
        if not self.path or len(self.path) <= 1:
            # Pour le leader, c'est déjà géré
            # Pour un suiveur, vérifier s'il doit se positionner derrière son leader
            if not self.is_leader and self.following_robot_id in self.other_robot_positions:
                self.position_behind_leader()

            self.get_logger().info(f'Robot {self.robot_id} reached path endpoint')
            if self.path_timer:
                self.path_timer.cancel()
            self.movement_in_progress = False
            return

        # Continuer normalement si pas arrivé
        next_point = self.path.pop(0)
        dx = next_point.x - self.position.x
        dy = next_point.y - self.position.y
        if abs(dx) > 1e-3 or abs(dy) > 1e-3:
            self.orientation = math.atan2(dy, dx)
        self.position = next_point
        self.publish_position()
        self.publish_path_marker()

    def position_behind_leader(self):
        """Place le robot exactement derrière le robot qu'il suit en formant un convoi horizontal"""
        if self.following_robot_id <= 0 or self.following_robot_id not in self.other_robot_positions:
            return

        leader_pos = self.other_robot_positions[self.following_robot_id]

        # Si le leader est arrivé à destination, on forme un convoi horizontal
        if self.following_robot_id in self.arrived_robots and self.final_destination:
            # Direction du convoi: horizontale vers la gauche (-1, 0)
            convoy_direction = (-1, 0)

            # Calculer la position dans le convoi basée sur l'ID du robot
            relative_position = self.robot_id - self.following_robot_id
            if relative_position <= 0:  # Si l'ID est inférieur au leader, on le place après
                relative_position = self.num_robots

            # Distance entre chaque robot dans le convoi
            convoy_spacing = 5.0  # Distance entre chaque robot

            # Calculer la position dans le convoi
            behind_x = leader_pos.x + convoy_direction[0] * (relative_position * convoy_spacing)
            behind_y = leader_pos.y + convoy_direction[1] * (relative_position * convoy_spacing)

            behind_pos = Vec2D(behind_x, behind_y)

            # Vérifier si cette position est valide (pas dans un mur)
            if self.map.is_wall(behind_pos):
                # Chercher une position valide à proximité en spirale
                found_valid_pos = False
                for radius in range(1, 15):  # Chercher dans un rayon croissant
                    for dx in range(-radius, radius + 1):
                        for dy in range(-radius, radius + 1):
                            # Vérifier seulement les points sur le "périmètre" de la spirale
                            if abs(dx) == radius or abs(dy) == radius:
                                test_pos = Vec2D(behind_x + dx, behind_y + dy)
                                if not self.map.is_wall(test_pos):
                                    # Trouver le chemin A* pour vérifier si la position est accessible
                                    path = self.astar.find_path(self.position, test_pos)
                                    if path:
                                        behind_pos = test_pos
                                        found_valid_pos = True
                                        break
                        if found_valid_pos:
                            break
                    if found_valid_pos:
                        break
        else:
            # Comportement normal quand le leader n'est pas encore arrivé
            leader_orientation = self.other_robot_orientations.get(self.following_robot_id, 0.0)

            # Positionner exactement derrière le leader
            behind_x = leader_pos.x - math.cos(leader_orientation) * self.follow_distance
            behind_y = leader_pos.y - math.sin(leader_orientation) * self.follow_distance

            behind_pos = Vec2D(behind_x, behind_y)

            # Vérifier si cette position est valide (pas dans un mur)
            if self.map.is_wall(behind_pos):
                # Chercher une position valide dans un arc autour du leader
                for angle_offset in range(-90, 91, 10):  # De -90° à +90° par pas de 10°
                    test_angle = leader_orientation + math.radians(angle_offset)
                    test_x = leader_pos.x - math.cos(test_angle) * self.follow_distance
                    test_y = leader_pos.y - math.sin(test_angle) * self.follow_distance
                    test_pos = Vec2D(test_x, test_y)

                    if not self.map.is_wall(test_pos):
                        behind_pos = test_pos
                        break

        # Déplacer le robot à cette position
        self.position = behind_pos

        # Orienter le robot correctement
        if self.following_robot_id in self.arrived_robots and self.final_destination:
            # Si en convoi, tous les robots ont la même orientation horizontale
            self.orientation = math.atan2(convoy_direction[1], convoy_direction[0])
        else:
            # Sinon, orienter vers le leader
            self.orientation = math.atan2(leader_pos.y - behind_pos.y,
                                        leader_pos.x - behind_pos.x)

        self.publish_position()

        self.get_logger().info(f'Robot {self.robot_id} positioned behind Robot {self.following_robot_id} '
                              f'at ({behind_pos.x:.2f}, {behind_pos.y:.2f})')

    def publish_position(self):
        if not self.position:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.position.x)
        msg.pose.position.y = float(self.position.y)
        msg.pose.position.z = 0.0
        cy = math.cos(self.orientation * 0.5)
        sy = math.sin(self.orientation * 0.5)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy
        self.position_pub.publish(msg)

    def use_policy_for_movement(self):
        """Utilise la politique calculée pour déterminer la prochaine action"""
        if not self.current_policy:
            self.get_logger().warn("No policy available, falling back to A*")
            self.recalculate_path()
            return

        # Trouver l'état le plus proche dans la politique pour ce robot
        closest_state = self._find_closest_policy_state(self.position)

        if not closest_state or self.position.distance_to(closest_state) > 5.0:
            self.get_logger().warn(f"No close policy state found, using A*")
            self.recalculate_path()
            return

        # Obtenir l'action recommandée
        action = self.current_policy[closest_state]
        self.get_logger().info(f"Using policy action: {action} from state {closest_state.x}, {closest_state.y}")

        # Utiliser l'environnement stochastique pour déterminer le prochain état
        next_pos = self.env.sample_next_state(self.position, action)

        # Si next_pos est un mur, utiliser la position actuelle
        if self.map.is_wall(next_pos):
            self.get_logger().warn("Policy led to wall, adjusting...")
            next_pos = self.position

        # Créer un chemin simple avec cette position
        self.path = [next_pos]
        self._start_timer = self.create_timer(0.5, self._delayed_start_movement)

    def _find_closest_policy_state(self, position):
        """Trouve l'état le plus proche dans la politique actuelle"""
        closest_state = None
        min_distance = float('inf')

        for state in self.current_policy.keys():
            distance = position.distance_to(state)
            if distance < min_distance:
                min_distance = distance
                closest_state = state

        return closest_state

    def update_follow_target(self):
        """Met à jour la cible à suivre si le robot est un suiveur"""
        if self.following_robot_id <= 0 or self.stopped or self.is_leader or self.has_reached_destination:
            return

        if self.following_robot_id not in self.other_robot_positions:
            return

        # Position du robot à suivre
        leader_pos = self.other_robot_positions[self.following_robot_id]

        # Si on est déjà en mouvement et assez proche, ne pas recalculer
        if self.movement_in_progress and leader_pos.distance_to(self.position) < self.follow_distance:
            return

        # Si le leader est arrivé, se positionner derrière lui
        if self.following_robot_id in self.arrived_robots:
            self.position_behind_leader()
            return

        # Calculer une position derrière le leader
        leader_orientation = self.other_robot_orientations.get(self.following_robot_id, 0.0)

        # Position derrière le leader (dans la direction opposée à son orientation)
        follow_x = leader_pos.x - math.cos(leader_orientation) * self.follow_distance
        follow_y = leader_pos.y - math.sin(leader_orientation) * self.follow_distance

        follow_target = Vec2D(follow_x, follow_y)

        # Si la cible est un mur, trouver une position libre à proximité
        if self.map.is_wall(follow_target):
            # Chercher dans un rayon autour de la position idéale
            for angle in range(0, 360, 30):  # Tous les 30 degrés
                rad = math.radians(angle)
                test_x = leader_pos.x + math.cos(rad) * self.follow_distance
                test_y = leader_pos.y + math.sin(rad) * self.follow_distance
                test_pos = Vec2D(test_x, test_y)

                if not self.map.is_wall(test_pos):
                    follow_target = test_pos
                    break

        # Si on est assez loin de la cible actuelle, recalculer le chemin
        if not self.target or follow_target.distance_to(self.target) > 2.0:
            self.target = follow_target
            self.recalculate_path()

    def publish_path_marker(self):
        """Publie le chemin du robot pour visualisation dans RViz"""
        if not self.path:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"robot_{self.robot_id}_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Définir l'apparence du chemin
        marker.scale.x = 0.1  # Largeur de la ligne

        # Couleur selon le robot
        if self.robot_id == 1:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif self.robot_id == 2:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif self.robot_id == 3:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        marker.color.a = 1.0  # Alpha (opacité)

        # Commencer par la position actuelle
        start_point = Point()
        start_point.x = self.position.x
        start_point.y = self.position.y
        start_point.z = 0.1
        marker.points.append(start_point)

        # Ajouter tous les points du chemin
        for point in self.path:
            p = Point()
            p.x = point.x
            p.y = point.y
            p.z = 0.1  # Légèrement au-dessus du sol
            marker.points.append(p)

        # Publier le marqueur
        self.path_marker_pub.publish(marker)

    def parse_policy(self, policy_str, is_jesp):
        """Parse la politique reçue du nœud JESP ou Best Response"""
        try:
            # Extraire les informations pour ce robot
            robot_pattern = f"Robot {self.robot_id}:(.*?)(?:Robot|$)"
            robot_match = re.search(robot_pattern, policy_str, re.DOTALL)

            if not robot_match:
                self.get_logger().warn(f"No policy found for Robot {self.robot_id}")
                return

            robot_policy_str = robot_match.group(1)

            # Extraire les positions et actions
            position_pattern = r"Position \(([0-9.]+), ([0-9.]+)\): ([a-z]+)"
            positions_actions = re.findall(position_pattern, robot_policy_str)

            # Créer le dictionnaire de politique
            policy = {}
            for x_str, y_str, action in positions_actions:
                try:
                    x = float(x_str)
                    y = float(y_str)
                    pos = Vec2D(x, y)
                    policy[pos] = action
                except ValueError:
                    self.get_logger().warn(f"Error parsing position: ({x_str}, {y_str})")

            # Mettre à jour la politique appropriée
            if is_jesp:
                self.jesp_policy = policy
                self.get_logger().info(f"Updated JESP policy with {len(policy)} states")
            else:
                self.best_response_policy = policy
                self.get_logger().info(f"Updated Best Response policy with {len(policy)} states")

        except Exception as e:
            self.get_logger().error(f"Error parsing policy: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    # MODIFIÉ: Utilisation des positions initiales correctes pour les 3 robots
    robot_nodes = [
        RobotController(1, 20.0, 20.0),
        RobotController(2, 35.0, 35.0),
        RobotController(3, 60.0, 20.0),
    ]
    executor = rclpy.executors.MultiThreadedExecutor()
    for r in robot_nodes:
        executor.add_node(r)
    try:
        executor.spin()
    finally:
        for r in robot_nodes:
            r.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
