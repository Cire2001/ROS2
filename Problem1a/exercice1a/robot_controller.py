# robot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.srv import GetMap
from exercice1.map2d import Map2D
from exercice1.vec2d import Vec2D
from exercice1.astar import AStar
import math
import random
import sys
import time

class RobotController(Node):
    def __init__(self, robot_id=1):
        super().__init__(f'robot_{robot_id}')

        self.robot_id = robot_id
        self.position = None
        self.target = None
        self.path = []
        self.current_path_index = 0

        # Récupération de la carte
        self.cli = self.create_client(GetMap, '/map_server/map')

        # Attendre le service avec un timeout global
        service_available = False
        timeout_time = time.time() + 10.0  # 10 secondes de timeout

        while not service_available and time.time() < timeout_time:
            service_available = self.cli.wait_for_service(timeout_sec=1.0)
            if not service_available:
                self.get_logger().info('service not available, waiting again...')

        if not service_available:
            self.get_logger().error('Map service not available after timeout. Continuing without map.')
            # Créer une carte vide par défaut
            from nav_msgs.msg import OccupancyGrid
            empty_map = OccupancyGrid()
            empty_map.info.width = 100
            empty_map.info.height = 100
            empty_map.info.resolution = 0.2
            empty_map.data = [0] * (100 * 100)
            self.map = Map2D(empty_map)
        else:
            self.req = GetMap.Request()
            self.map_response = self.get_map()
            self.map = Map2D(self.map_response.map)
            self.get_logger().info('Map service connected successfully')

        # Initialiser A*
        self.astar = AStar(self.map)

        # Souscriptions
        self.create_subscription(
            Point,
            f'/robot_{robot_id}/target_position',
            self.target_callback,
            10)

        # MODIFICATION: suppression de la souscription à /clicked_point
        # pour éviter le conflit avec destination_manager

        # Publications
        self.position_pub = self.create_publisher(
            PoseStamped,
            f'/robot_{robot_id}/current_position',
            10)

        self.global_pos_pub = self.create_publisher(
            PoseStamped,
            '/robot_position',
            10)

        # Timer
        self.timer = self.create_timer(0.2, self.timer_callback)

        # Initialiser automatiquement la position du robot (pour le débogage)
        self.position = Vec2D(20.0 + robot_id * 5, 20.0)  # Positions différentes pour chaque robot
        self.get_logger().info(f'Robot position initialized automatically: ({self.position.x}, {self.position.y})')
        self.publish_position()

        self.get_logger().info(f'Robot {robot_id} Controller started')

    def get_map(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def clicked_point_callback(self, msg):
        # Cette méthode n'est plus utilisée mais conservée
        # au cas où nous voudrions la réactiver plus tard
        # Initialiser la position du robot en cliquant sur la carte
        point = Vec2D(msg.point.x, msg.point.y)

        if not self.map.is_wall(point):
            self.position = point
            self.get_logger().info(f'Robot position initialized: ({point.x}, {point.y})')

            # Publier la position initiale
            self.publish_position()
        else:
            self.get_logger().warn('Cannot initialize position: location is a wall')

    def target_callback(self, msg):
        # Recevoir une nouvelle position cible
        new_target = Vec2D(msg.x, msg.y)

        if self.target is None or self.target.x != new_target.x or self.target.y != new_target.y:
            self.target = new_target
            self.get_logger().info(f'New target received: ({new_target.x}, {new_target.y})')

            # Calculer un nouveau chemin si nous avons une position
            if self.position is not None:
                self.calculate_path()

    def calculate_path(self):
        # Calculer un chemin vers la cible en utilisant A*
        self.path = self.astar.find_path(self.position, self.target)

        if self.path:
            self.current_path_index = 0
            self.get_logger().info(f'Path calculated with {len(self.path)} steps')
        else:
            self.get_logger().warn('No path found to target')

    def timer_callback(self):
        if self.position is None or self.path is None or len(self.path) == 0:
            return

        # Si nous avons atteint la fin du chemin
        if self.current_path_index >= len(self.path) - 1:
            return

        # Avancer au prochain point du chemin
        self.current_path_index += 1
        self.position = self.path[self.current_path_index]

        # Publier la nouvelle position
        self.publish_position()

    def publish_position(self):
        # Créer et publier un message PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = self.position.x
        msg.pose.position.y = self.position.y
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        self.position_pub.publish(msg)
        self.global_pos_pub.publish(msg)
        self.get_logger().info(f'Published position: ({self.position.x}, {self.position.y})')

def main(args=None):
    print("Starting robot_controller main function")
    try:
        rclpy.init(args=args)
        print("ROS2 initialized")

        # Créer plusieurs robots
        robots = []
        for i in range(1, 4):  # 3 robots
            node = RobotController(robot_id=i)
            robots.append(node)
            print(f"Robot {i} controller node created")

        # Utiliser MultiThreadedExecutor pour exécuter plusieurs nœuds
        executor = rclpy.executors.MultiThreadedExecutor()
        for robot in robots:
            executor.add_node(robot)

        print("Starting executor.spin()")
        executor.spin()
        print("Spin completed")
    except Exception as e:
        print(f"Error in main function: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up")
        if 'robots' in locals():
            for robot in robots:
                robot.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete")

if __name__ == '__main__':
    main()
