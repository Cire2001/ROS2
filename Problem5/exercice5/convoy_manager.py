#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker, MarkerArray

from exercice5.vec2d import Vec2D
from exercice5.map2d import Map2D
from exercice5.astar import AStar

class ConvoyManager(Node):
    """Manage a convoy of robots"""

    def __init__(self):
        super().__init__('convoy_manager')

        # --- Client du service GetMap pour récupérer l'OccupancyGrid ---
        self.get_logger().info('Waiting for /map_server/map service...')
        self._map_client = self.create_client(GetMap, '/map_server/map')
        req = GetMap.Request()
        while not self._map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map service not available, retrying...')
        future = self._map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        map_msg = future.result().map

        # --- Construction de notre wrapper Map2D ---
        self.map = Map2D(map_msg)
        self.get_logger().info("Map loaded successfully")

        # --- Nombre de robots gérés ---
        self.num_robots = 3
        self.get_logger().info(f"Managing {self.num_robots} robots")

        # --- Destination courante (Vec2D) ---
        self.destination = Vec2D(0.0, 0.0)

        # --- Positions des robots ---
        self.robot_positions = {}

        # --- Suivi des rôles (leader, followers) ---
        self.leader_id = -1
        self.followers = {}  # robot_id -> robot_to_follow_id

        # --- Publisher de la destination finale ---
        self.destination_publisher = self.create_publisher(
            Point, '/set_destination', 10
        )

        # --- Publishers pour les rôles des robots ---
        self.leader_publisher = self.create_publisher(
            Int32, '/convoy_leader', 10
        )

        self.follow_publishers = {}
        for i in range(1, self.num_robots + 1):
            self.follow_publishers[i] = self.create_publisher(
                Int32, f'/robot_{i}/follow_robot', 10
            )

        # --- Subscribers aux positions des robots ---
        for i in range(1, self.num_robots + 1):
            self.create_subscription(
                Point,
                f'/robot_{i}/current_position',
                lambda msg, robot_id=i: self.robot_position_callback(msg, robot_id),
                10
            )

        # --- Subscribers aux politiques calculées ---
        self.jesp_subscription = self.create_subscription(
            String, '/jesp_policy', self.jesp_callback, 10
        )
        self.best_response_subscription = self.create_subscription(
            String, '/best_response_policy', self.best_response_callback, 10
        )

        # --- Publisher pour les chemins A* (visualisation) ---
        self.path_publisher = self.create_publisher(
            MarkerArray, '/astar_paths', 10
        )

        # --- Timer pour fixer la destination initiale ---
        self.get_logger().info("Setting up initial destination timer")
        self._startup_timer = self.create_timer(3.0, self._set_initial_destination)

        # --- Timer pour mettre à jour les rôles ---
        self.role_timer = self.create_timer(2.0, self.update_convoy_roles)

        self.get_logger().info("Convoy Manager initialized successfully!")

    def _set_initial_destination(self) -> None:
        """Définit la destination initiale après 3 s, un cran à l'intérieur du coin Nord‐Est."""
        self.get_logger().info("Setting initial destination now…")
        self.destroy_timer(self._startup_timer)

        info = self.map.map.info
        # On décale d'une cellule depuis le bord : width-2 et height-2
        max_x = (info.width - 2) * info.resolution
        max_y = (info.height - 2) * info.resolution
        destination = Vec2D(max_x, max_y)

        self.set_destination(destination)
        self.get_logger().info(
            f"Initial destination set to ({destination.x:.2f}, {destination.y:.2f}) "
            "(one cell inside the grid)"
        )

    def set_destination(self, destination: Vec2D) -> None:
        """Change la destination et la publie immédiatement."""
        self.destination = destination
        self.publish_destination()

    def publish_destination(self) -> None:
        """Publie la destination sur `/set_destination`."""
        msg = Point()
        msg.x = self.destination.x
        msg.y = self.destination.y
        msg.z = 0.0
        self.get_logger().info(f"Publishing destination: ({msg.x:.2f}, {msg.y:.2f})")
        self.destination_publisher.publish(msg)

    def robot_position_callback(self, msg, robot_id):
        """Appelé quand un robot publie sa position."""
        pos = Vec2D(msg.x, msg.y)
        self.robot_positions[robot_id] = pos

        # Si tous les robots ont publié leur position, on peut mettre à jour les rôles
        if len(self.robot_positions) >= self.num_robots:
            self.update_convoy_roles()

    def update_convoy_roles(self):
        """Met à jour les rôles des robots dans le convoi selon la distance A*."""
        if not self.destination or len(self.robot_positions) < self.num_robots:
            return

        # Utilisez A* pour calculer la vraie distance vers la destination
        astar = AStar(self.map)
        distances = {}
        paths = {}

        for robot_id, position in self.robot_positions.items():
            path = astar.find_path(position, self.destination)
            # Distance = longueur du chemin (ou infini si pas de chemin)
            distances[robot_id] = len(path) if path else float('inf')
            paths[robot_id] = path

        # Visualiser les chemins A*
        self.visualize_paths(paths)

        # Le leader est le robot avec le chemin le plus court vers la destination
        sorted_robots = sorted(distances.items(), key=lambda x: x[1])
        new_leader = sorted_robots[0][0]

        if new_leader != self.leader_id:
            self.get_logger().info(f"New leader: Robot {new_leader}")
            self.leader_id = new_leader
            # Publier le nouveau leader
            leader_msg = Int32()
            leader_msg.data = new_leader
            self.leader_publisher.publish(leader_msg)

        # Pour chaque robot qui n'est pas le leader
        followers = {}
        for robot_id in range(1, self.num_robots + 1):
            if robot_id != new_leader:
                # Trouver le robot le plus proche qui est plus proche de l'objectif
                best_follow = None
                min_distance = float('inf')

                for other_id, other_dist in sorted_robots:
                    # Ne considérer que les robots plus proches de l'objectif
                    if distances[other_id] < distances[robot_id]:
                        # Calculer la distance entre ce robot et l'autre
                        current_dist = self.robot_positions[robot_id].distance_to(self.robot_positions[other_id])
                        if current_dist < min_distance:
                            min_distance = current_dist
                            best_follow = other_id

                if best_follow is not None:
                    followers[robot_id] = best_follow
                else:
                    # Si aucun robot n'est plus proche, suivre le leader
                    followers[robot_id] = new_leader

        # Publier les instructions de suivi pour chaque robot
        for follower_id, follow_id in followers.items():
            if self.followers.get(follower_id) != follow_id:
                self.get_logger().info(f"Robot {follower_id} suivra Robot {follow_id}")
                msg = Int32()
                msg.data = follow_id
                self.follow_publishers[follower_id].publish(msg)

        self.followers = followers

        # Le leader doit savoir qu'il est leader
        leader_follow_msg = Int32()
        leader_follow_msg.data = -1  # -1 signifie "ne suit personne"
        self.follow_publishers[new_leader].publish(leader_follow_msg)

    def visualize_paths(self, paths):
        """Publie les chemins A* pour visualisation dans RViz."""
        marker_array = MarkerArray()
        marker_id = 0

        # Créer un marqueur pour chaque robot
        for robot_id, path in paths.items():
            if not path:
                continue

            # Créer un marqueur pour le chemin
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"robot_{robot_id}_path"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # Définir l'apparence du chemin
            marker.scale.x = 0.1  # Largeur de la ligne

            # Couleur différente pour chaque robot
            if robot_id == 1:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif robot_id == 2:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif robot_id == 3:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0  # Alpha (opacité)

            # Ajouter les points du chemin
            for point in path:
                p = Point()
                p.x = point.x
                p.y = point.y
                p.z = 0.1  # Légèrement au-dessus du sol
                marker.points.append(p)

            marker_array.markers.append(marker)
            marker_id += 1

        # Publier tous les marqueurs
        if marker_array.markers:
            self.path_publisher.publish(marker_array)

    def jesp_callback(self, msg: String) -> None:
        """Reçu quand JESP publie une nouvelle politique."""
        self.get_logger().info("Received JESP policy update")

    def best_response_callback(self, msg: String) -> None:
        """Reçu quand Best Response publie une nouvelle politique."""
        self.get_logger().info("Received Best Response policy update")

def main(args=None):
    rclpy.init(args=args)
    node = ConvoyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Convoy Manager")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
