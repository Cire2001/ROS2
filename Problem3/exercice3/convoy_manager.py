# convoy_manager.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32, Bool
from nav_msgs.srv import GetMap
from exercice3.vec2d import Vec2D
from exercice3.map2d import Map2D
from exercice3.astar import AStar
import time

class ConvoyManager(Node):
    def __init__(self):
        super().__init__('convoy_manager')
        self.num_robots = 5
        self.destination = None
        self.robot_positions = {}
        self.robot_distances = {}
        self.leader_id = -1
        self.convoy_established = False

        # Obtenir la carte
        self.get_logger().info('Convoy Manager waiting for map service...')
        self.cli = self.create_client(GetMap, '/map_server/map')
        self.req = GetMap.Request()
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map service not available, waiting again...')

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.map_response = self.future.result()
        self.map = Map2D(self.map_response.map)
        self.astar = AStar(self.map)

        map_info = self.map_response.map.info
        self.get_logger().info(f"Map dimensions: {map_info.width} x {map_info.height}")
        self.get_logger().info(f"Map resolution: {map_info.resolution}")

        # Publications et souscriptions
        self.destination_marker_pub = self.create_publisher(
            Marker, '/convoy_destination_marker', 10)

        for i in range(1, self.num_robots + 1):
            self.create_subscription(
                PoseStamped,
                f'/robot_{i}/current_position',
                lambda msg, robot_id=i: self.robot_position_callback(msg, robot_id),
                10)

        self.create_subscription(
            Point, '/set_destination', self.set_destination_callback, 10)

        self.target_pubs = {}
        self.follow_robot_pubs = {}
        self.stop_pubs = {}

        for i in range(1, self.num_robots + 1):
            self.target_pubs[i] = self.create_publisher(
                Point, f'/robot_{i}/target_position', 10)
            self.follow_robot_pubs[i] = self.create_publisher(
                Int32, f'/robot_{i}/follow_robot', 10)
            self.stop_pubs[i] = self.create_publisher(
                Bool, f'/robot_{i}/stop_command', 10)

        self.marker_timer = self.create_timer(1.0, self.publish_destination_marker)
        self.get_logger().info('Convoy Manager started')

    def robot_position_callback(self, msg, robot_id):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_positions[robot_id] = Vec2D(x, y)

    def set_destination_callback(self, msg):
        # Arrêter tous les robots
        self.stop_all_robots()
        time.sleep(0.5)

        self.destination = Vec2D(msg.x, msg.y)
        self.get_logger().info(f'Convoy destination set to: ({msg.x}, {msg.y})')
        self.leader_id = -1
        self.robot_distances = {}
        self.convoy_established = False
        self.publish_destination_marker()

        # Attendre les positions des robots
        start_waiting = time.time()
        while len(self.robot_positions) < self.num_robots:
            if time.time() - start_waiting > 3.0:
                self.get_logger().warn(f"Timeout waiting for robot positions. Got {len(self.robot_positions)}/{self.num_robots}")
                break
            time.sleep(0.1)

        # Organiser le convoi
        self.organize_convoy()

    def stop_all_robots(self):
        stop_msg = Bool()
        stop_msg.data = True
        for robot_id in range(1, self.num_robots + 1):
            self.stop_pubs[robot_id].publish(stop_msg)
        self.get_logger().info("Stop command sent to all robots")

    def calculate_path_distances(self):
        """Calcule la distance réelle (chemin) de chaque robot à la destination"""
        if not self.destination:
            return False

        if len(self.robot_positions) < self.num_robots:
            self.get_logger().warn(f"Missing robot positions. Got {len(self.robot_positions)}/{self.num_robots}")
            return False

        # Calculer la distance de chemin pour chaque robot
        for robot_id, position in self.robot_positions.items():
            path = self.astar.find_path(position, self.destination)
            if path:
                # Calculer la longueur du chemin
                path_length = 0.0
                prev_point = position
                for point in path:
                    path_length += prev_point.distance_to(point)
                    prev_point = point

                self.robot_distances[robot_id] = path_length
                self.get_logger().info(f"Robot {robot_id} path distance: {path_length:.2f}")
            else:
                self.get_logger().warn(f"No path found for Robot {robot_id}")
                self.robot_distances[robot_id] = float('inf')  # Distance infinie si pas de chemin

        return True

    def organize_convoy(self):
        """Organise le convoi en désignant un leader et en faisant suivre les autres robots"""
        if not self.destination:
            self.get_logger().warn('No destination set')
            return

        # Si le convoi est déjà établi, ne pas le réorganiser
        if self.convoy_established:
            self.get_logger().info("Convoy already established, maintaining formation")
            return

        # Calculer les distances de chemin
        if not self.calculate_path_distances():
            self.get_logger().warn("Failed to calculate path distances")
            return

        # Trouver le robot le plus proche de la destination (leader)
        min_distance = float('inf')
        for robot_id, distance in self.robot_distances.items():
            if distance < min_distance:
                min_distance = distance
                self.leader_id = robot_id

        if self.leader_id == -1:
            self.get_logger().error("Failed to determine leader robot")
            return

        self.get_logger().info(f"Selected Robot {self.leader_id} as leader (distance: {min_distance:.2f})")

        # 1. D'abord configurer tous les robots
        for robot_id in range(1, self.num_robots + 1):
            if robot_id == self.leader_id:
                # Le leader est autonome et va directement à la destination
                follow_msg = Int32()
                follow_msg.data = -1  # Ne suit personne
                self.follow_robot_pubs[robot_id].publish(follow_msg)
                self.get_logger().info(f"Setting Robot {robot_id} as leader (autonomous)")
            else:
                # Les autres robots suivent le leader
                follow_msg = Int32()
                follow_msg.data = self.leader_id
                self.follow_robot_pubs[robot_id].publish(follow_msg)
                self.get_logger().info(f"Setting Robot {robot_id} to follow leader (Robot {self.leader_id})")

        # 2. Petit délai pour s'assurer que les configurations sont appliquées
        time.sleep(0.5)

        # 3. Envoyer la destination au leader uniquement
        target_msg = Point()
        target_msg.x = float(self.destination.x)
        target_msg.y = float(self.destination.y)
        target_msg.z = 0.0
        self.target_pubs[self.leader_id].publish(target_msg)
        self.get_logger().info(f"Sending destination ({self.destination.x}, {self.destination.y}) to leader Robot {self.leader_id}")

        # 4. Démarrer tous les robots
        for robot_id in range(1, self.num_robots + 1):
            stop_msg = Bool()
            stop_msg.data = False
            self.stop_pubs[robot_id].publish(stop_msg)
            self.get_logger().info(f"Starting Robot {robot_id}")

        self.convoy_established = True
        self.get_logger().info("Convoy established and moving")

    def publish_destination_marker(self):
        if not self.destination:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "convoy_destination"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.destination.x)
        marker.pose.position.y = float(self.destination.y)
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 4.0
        marker.scale.y = 4.0
        marker.scale.z = 4.0
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.0
        marker.color.a = 0.7
        self.destination_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ConvoyManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
