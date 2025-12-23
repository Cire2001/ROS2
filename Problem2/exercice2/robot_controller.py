# robot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool
from exercice2.map2d import Map2D
from exercice2.vec2d import Vec2D
from exercice2.astar import AStar

class SingleRobotController(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}')

        self.robot_id = robot_id
        self.position = None
        self.target = None
        self.path = []
        self.stopped = False
        self.path_timer = None

        # Positions initiales
        if robot_id == 1:
            self.position = Vec2D(20.0, 20.0)
        elif robot_id == 2:
            self.position = Vec2D(35.0, 35.0)
        else:
            self.position = Vec2D(60.0, 20.0)

        # Publications
        self.position_pub = self.create_publisher(
            PoseStamped,
            f'/robot_{robot_id}/current_position',
            10)

        # Souscriptions
        self.target_sub = self.create_subscription(
            Point,
            f'/robot_{robot_id}/target_position',
            self.target_callback,
            10)

        # Souscription à la commande d'arrêt
        self.stop_sub = self.create_subscription(
            Bool,
            f'/robot_{robot_id}/stop_command',
            self.stop_callback,
            10)

        # Timer pour publier régulièrement la position
        self.position_timer = self.create_timer(0.5, self.publish_position)

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

        # Publier la position initiale
        self.publish_position()
        self.get_logger().info(f'Robot {robot_id} initialized at ({self.position.x}, {self.position.y})')

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().warn(f'***** ROBOT {self.robot_id} RECEIVED STOP COMMAND *****')
            self.stopped = True

            # Annuler le timer de suivi de chemin s'il existe
            if self.path_timer:
                self.path_timer.cancel()
                self.path_timer = None

            self.path = []  # Effacer le chemin
            self.target = None  # Effacer la cible

            # Publier immédiatement la position pour confirmer que le robot est arrêté
            self.publish_position()

    def target_callback(self, msg):
        # Ignorer toutes les commandes si le robot est arrêté
        if self.stopped:
            self.get_logger().info(f'Robot {self.robot_id} is stopped - ignoring new target')
            return

        new_target = Vec2D(msg.x, msg.y)

        # Vérifier si la cible a changé
        if self.target and abs(self.target.x - new_target.x) < 0.1 and abs(self.target.y - new_target.y) < 0.1:
            return  # Même cible, ignorer

        self.target = new_target
        self.get_logger().info(f'Robot {self.robot_id} received new target: ({new_target.x}, {new_target.y})')

        # Calculer le chemin
        self.path = self.astar.find_path(self.position, self.target)
        if self.path:
            self.get_logger().info(f'Path calculated with {len(self.path)} steps')

            # Annuler l'ancien timer s'il existe
            if self.path_timer:
                self.path_timer.cancel()

            # Démarrer un timer pour suivre le chemin
            self.path_timer = self.create_timer(0.2, self.follow_path_callback)
        else:
            self.get_logger().warn('No path found to target')

    def follow_path_callback(self):
        # Vérifier si le robot est arrêté
        if self.stopped:
            if self.path_timer:
                self.path_timer.cancel()
                self.path_timer = None
            return

        if not self.path or len(self.path) <= 1:
            self.get_logger().info(f'Robot {self.robot_id} reached destination')
            if self.path_timer:
                self.path_timer.cancel()
                self.path_timer = None
            return

        # Avancer au prochain point
        next_point = self.path.pop(0)
        self.position = next_point
        self.publish_position()

    def publish_position(self):
        if not self.position:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = float(self.position.x)
        msg.pose.position.y = float(self.position.y)
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        self.position_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Créer les robots en tant que nœuds séparés
    robot1 = SingleRobotController(1)
    robot2 = SingleRobotController(2)
    robot3 = SingleRobotController(3)

    # Utiliser MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot1)
    executor.add_node(robot2)
    executor.add_node(robot3)

    try:
        executor.spin()
    finally:
        robot1.destroy_node()
        robot2.destroy_node()
        robot3.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
