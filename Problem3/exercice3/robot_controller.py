# robot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Pose
from visualization_msgs.msg import Marker
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool, Int32, ColorRGBA
from exercice3.map2d import Map2D
from exercice3.vec2d import Vec2D
from exercice3.astar import AStar
import math
import time
import random

class RobotController(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}')

        # Basic properties
        self.robot_id = robot_id
        self.position = None
        self.target = None
        self.path = None  # Using None instead of empty list for clarity
        self.stopped = False
        self.following_robot_id = -1
        self.leader_position = None
        self.orientation = 0.0
        self.follow_distance = 8.0
        self.step_index = 0
        self.move_speed = 1.0
        self.consecutive_blocks = 0
        self.last_path_time = time.time()
        self.path_recalculation_cooldown = 2.0
        self.distance_threshold = 5.0

        # Initial positions
        if robot_id == 1:
            self.position = Vec2D(20.0, 20.0)
        elif robot_id == 2:
            self.position = Vec2D(35.0, 35.0)
        elif robot_id == 3:
            self.position = Vec2D(60.0, 20.0)
        elif robot_id == 4:
            self.position = Vec2D(10.0, 50.0)
        elif robot_id == 5:
            self.position = Vec2D(70.0, 70.0)

        # Robot colors
        self.colors = {
            1: ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Red
            2: ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Green
            3: ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Blue
            4: ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # Yellow
            5: ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)   # Magenta
        }

        # Publishers
        self.position_pub = self.create_publisher(
            PoseStamped, f'/robot_{robot_id}/current_position', 10)
        self.marker_pub = self.create_publisher(
            Marker, f'/robot_{robot_id}/visualization_marker', 10)
        self.position_point_pub = self.create_publisher(
            Point, f'/robot_{robot_id}/position', 10)

        # Subscriptions
        self.target_sub = self.create_subscription(
            Point, f'/robot_{robot_id}/target_position', self.target_callback, 10)
        self.stop_sub = self.create_subscription(
            Bool, f'/robot_{robot_id}/stop_command', self.stop_callback, 10)
        self.follow_robot_sub = self.create_subscription(
            Int32, f'/robot_{robot_id}/follow_robot', self.follow_robot_callback, 10)

        # Timers
        self.position_timer = self.create_timer(0.2, self.publish_position)
        self.marker_timer = self.create_timer(0.2, self.publish_markers)

        # Get the map
        self.get_logger().info(f'Robot {robot_id} waiting for map service...')
        self.cli = self.create_client(GetMap, '/map_server/map')
        self.req = GetMap.Request()
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map service not available, waiting again...')

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.map_response = self.future.result()
        self.map = Map2D(self.map_response.map)
        self.astar = AStar(self.map)

        self.get_logger().info(f'Robot {robot_id} received map data')
        self.publish_position()
        self.get_logger().info(f'Robot {robot_id} initialized at ({self.position.x}, {self.position.y})')

        # Movement timer for continuous movement
        self.movement_timer = self.create_timer(0.1, self.move_step)

    def follow_robot_callback(self, msg):
        """Callback to set robot to follow"""
        robot_to_follow = msg.data
        old_following = self.following_robot_id
        self.following_robot_id = robot_to_follow

        # Reset path when changing leader
        self.path = None
        self.step_index = 0
        self.consecutive_blocks = 0

        if robot_to_follow > 0:
            self.get_logger().info(f'Robot {self.robot_id} will follow robot {self.following_robot_id}')
            if old_following != robot_to_follow:
                self.create_subscription(
                    PoseStamped,
                    f'/robot_{robot_to_follow}/current_position',
                    self.leader_position_callback,
                    10)
        else:
            self.get_logger().info(f'Robot {self.robot_id} is autonomous (leader or isolated)')

    def leader_position_callback(self, msg):
        """Callback to update followed robot's position"""
        if self.following_robot_id <= 0:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        self.leader_position = Vec2D(x, y)

        if not self.stopped and time.time() - self.last_path_time > self.path_recalculation_cooldown:
            self.calculate_follow_path()
            self.last_path_time = time.time()

    def calculate_follow_path(self):
        """Calculate path to follow leader robot"""
        if not self.leader_position or not self.position:
            return

        # Calculate position behind leader with offset based on robot ID
        offset_angle = (self.robot_id * 72) % 360  # Different angles for different robots (360/5 = 72 degrees apart)
        rad = math.radians(offset_angle)
        direction = Vec2D(math.cos(rad), math.sin(rad)) * self.follow_distance

        target = Vec2D(
            self.leader_position.x + direction.x,
            self.leader_position.y + direction.y
        )

        # Check if target is in wall and try alternative positions if needed
        if self.map.is_wall(target, tolerance=0.5):
            found_valid_target = False
            for angle_offset in [0, 45, 90, 135, 180, 225, 270, 315]:
                test_angle = (offset_angle + angle_offset) % 360
                rad = math.radians(test_angle)
                test_dir = Vec2D(math.cos(rad), math.sin(rad)) * self.follow_distance
                test_target = Vec2D(
                    self.leader_position.x + test_dir.x,
                    self.leader_position.y + test_dir.y
                )

                if not self.map.is_wall(test_target, tolerance=0.5):
                    target = test_target
                    found_valid_target = True
                    break

            if not found_valid_target:
                # Gradually reduce follow distance if no valid position found
                self.follow_distance = max(3.0, self.follow_distance * 0.8)
                return

        self.target = target
        calculated_path = self.astar.find_path(self.position, self.target)

        if calculated_path and len(calculated_path) > 0:
            self.path = calculated_path
            self.step_index = 0
            self.consecutive_blocks = 0
            path_length = len(self.path)
            self.get_logger().info(f'Robot {self.robot_id} - Follow path calculated with {path_length} steps')
        else:
            self.path = None  # Explicitly set to None, not empty list
            self.step_index = 0
            self.get_logger().warn(f'Robot {self.robot_id} - No path found to follow Robot {self.following_robot_id}')
            # Try to find a more direct path next time
            self.follow_distance = max(3.0, self.follow_distance * 0.8)

    def stop_callback(self, msg):
        """Callback to stop/start robot"""
        self.stopped = msg.data
        if not self.stopped:
            self.last_path_time = 0  # Force immediate recalculation
            if self.following_robot_id > 0 and self.leader_position:
                self.calculate_follow_path()
            elif self.target:
                self.recalculate_path()

    def target_callback(self, msg):
        """Callback to set direct target"""
        if self.following_robot_id > 0:
            return

        self.target = Vec2D(msg.x, msg.y)

        if self.stopped:
            return

        self.get_logger().info(f'Robot {self.robot_id} received new target: ({self.target.x}, {self.target.y})')
        self.recalculate_path()

    def recalculate_path(self):
        """Recalculate path to direct target"""
        if not self.target or self.stopped:
            return

        if time.time() - self.last_path_time < self.path_recalculation_cooldown:
            return

        calculated_path = self.astar.find_path(self.position, self.target)
        self.last_path_time = time.time()

        if calculated_path and len(calculated_path) > 0:
            self.path = calculated_path
            self.step_index = 0
            self.consecutive_blocks = 0
            self.get_logger().info(f'Robot {self.robot_id} - Path calculated with {len(self.path)} steps')
        else:
            self.path = None  # Explicitly set to None, not empty list
            self.step_index = 0
            self.get_logger().warn(f'Robot {self.robot_id} - No path found to target')

    def try_escape_maneuver(self):
        """Try to escape when stuck"""
        self.get_logger().warn(f'Robot {self.robot_id} executing escape maneuver!')

        # Try 8 different directions
        escape_successful = False
        for angle in range(0, 360, 45):
            rad = math.radians(angle)
            escape_pos = Vec2D(
                self.position.x + math.cos(rad) * 3.0,
                self.position.y + math.sin(rad) * 3.0
            )

            if not self.map.is_wall(escape_pos, tolerance=0.3):
                self.get_logger().info(f'Robot {self.robot_id} found escape route at angle {angle}°')
                self.position = escape_pos
                self.consecutive_blocks = 0
                escape_successful = True
                break

        # If all else fails, make a small random movement
        if not escape_successful:
            random_angle = random.uniform(0, 2 * math.pi)
            random_distance = random.uniform(1.0, 2.0)
            self.get_logger().warn(f'Robot {self.robot_id} making random move to escape!')

            for test_distance in [random_distance, 0.8, 0.5, 0.3]:
                random_pos = Vec2D(
                    self.position.x + math.cos(random_angle) * test_distance,
                    self.position.y + math.sin(random_angle) * test_distance
                )

                if not self.map.is_wall(random_pos, tolerance=0.2):
                    self.position = random_pos
                    self.consecutive_blocks = 0
                    escape_successful = True
                    break

        # Recalculate path after moving
        if escape_successful:
            # Reset path and force recalculation
            self.path = None
            self.step_index = 0
            self.last_path_time = 0

            # Recalculate path from new position
            if self.following_robot_id > 0 and self.leader_position:
                self.calculate_follow_path()
            elif self.target:
                self.recalculate_path()

    def move_step(self):
        """Perform movement step along path"""
        # Safeguard: make sure we can move
        if self.stopped:
            return

        # Safeguard: check for valid path and index
        if not self.path or len(self.path) == 0:
            if time.time() - self.last_path_time > self.path_recalculation_cooldown:
                if self.following_robot_id > 0 and self.leader_position:
                    self.calculate_follow_path()
                elif self.target:
                    self.recalculate_path()
                self.last_path_time = time.time()
            return

        # Safeguard: check that step_index is valid
        if self.step_index >= len(self.path):
            self.step_index = min(self.step_index, len(self.path) - 1)
            if time.time() - self.last_path_time > self.path_recalculation_cooldown:
                if self.following_robot_id > 0 and self.leader_position:
                    self.calculate_follow_path()
                elif self.target:
                    self.recalculate_path()
                self.last_path_time = time.time()
            return

        # For followers, check if leader has moved significantly
        if self.following_robot_id > 0 and self.leader_position:
            if time.time() - self.last_path_time > self.path_recalculation_cooldown:
                current_distance = self.position.distance_to(self.leader_position)
                if current_distance > 15.0 or self.consecutive_blocks > 3:
                    self.calculate_follow_path()
                    self.last_path_time = time.time()
                    if not self.path or len(self.path) == 0:
                        return

        # Get next position - extra safeguard check
        if self.step_index >= len(self.path):
            self.get_logger().warn(f'Robot {self.robot_id} - step_index out of range, recalculating')
            self.path = None
            self.step_index = 0
            return

        next_point = self.path[self.step_index]

        # Calculate direction and distance
        dx = next_point.x - self.position.x
        dy = next_point.y - self.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate orientation
        if abs(dx) > 0.001 or abs(dy) > 0.001:
            self.orientation = math.atan2(dy, dx)

        # If close enough to next point, move to it and go to next
        if distance < self.move_speed:
            self.position = next_point
            self.step_index += 1
            self.consecutive_blocks = 0

            # Check if we've reached end of path
            if self.step_index >= len(self.path):
                if not self.following_robot_id > 0 and self.target:
                    if self.position.distance_to(self.target) < self.distance_threshold:
                        self.get_logger().info(f'Robot {self.robot_id} reached destination')
                    else:
                        self.recalculate_path()
                elif self.following_robot_id > 0 and self.leader_position:
                    # Recalculate follow path at end of current path
                    self.calculate_follow_path()
        else:
            # Otherwise, move a small step toward next point
            direction_x = dx / distance if distance > 0 else 0
            direction_y = dy / distance if distance > 0 else 0

            new_position = Vec2D(
                self.position.x + direction_x * self.move_speed,
                self.position.y + direction_y * self.move_speed
            )

            # Check with tolerance to avoid getting stuck
            if not self.map.is_wall(new_position, tolerance=0.5):
                self.position = new_position
                self.consecutive_blocks = 0
            else:
                # If blocked by wall, try to recover
                self.consecutive_blocks += 1
                self.get_logger().warn(f'Robot {self.robot_id} blocked by wall ({self.consecutive_blocks} times)')

                if self.consecutive_blocks > 5:
                    self.try_escape_maneuver()
                elif time.time() - self.last_path_time > self.path_recalculation_cooldown:
                    if self.following_robot_id > 0 and self.leader_position:
                        self.calculate_follow_path()
                    elif self.target:
                        self.recalculate_path()
                    self.last_path_time = time.time()

        # Publish new position immediately
        self.publish_position()

    def publish_position(self):
        """Publish robot's current position"""
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

        point_msg = Point()
        point_msg.x = float(self.position.x)
        point_msg.y = float(self.position.y)
        self.position_point_pub.publish(point_msg)

    def publish_markers(self):
        """Publish markers for RViz"""
        if not self.position:
            return

        # Robot marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = f"robot_{self.robot_id}"
        robot_marker.id = self.robot_id
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD

        robot_marker.pose.position.x = float(self.position.x)
        robot_marker.pose.position.y = float(self.position.y)
        robot_marker.pose.position.z = 0.5

        cy = math.cos(self.orientation * 0.5)
        sy = math.sin(self.orientation * 0.5)
        robot_marker.pose.orientation.w = cy
        robot_marker.pose.orientation.z = sy

        robot_marker.scale.x = 3.0
        robot_marker.scale.y = 1.5
        robot_marker.scale.z = 1.5

        robot_marker.color = self.colors[self.robot_id]
        robot_marker.lifetime.sec = 0

        self.marker_pub.publish(robot_marker)

        # Path marker
        if self.path and len(self.path) > 0 and self.step_index < len(self.path):
            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = f"path_{self.robot_id}"
            path_marker.id = self.robot_id * 100
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD

            path_points = []
            start_point = Point(x=float(self.position.x), y=float(self.position.y), z=0.1)
            path_points.append(start_point)

            for point in self.path[self.step_index:]:
                p = Point(x=float(point.x), y=float(point.y), z=0.1)
                path_points.append(p)

            path_marker.points = path_points
            path_marker.scale.x = 0.3

            color = self.colors[self.robot_id]
            path_marker.color.r = color.r
            path_marker.color.g = color.g
            path_marker.color.b = color.b
            path_marker.color.a = 0.4

            path_marker.lifetime.sec = 0
            self.marker_pub.publish(path_marker)

def main(args=None):
    rclpy.init(args=args)

    robot1 = RobotController(1)
    robot2 = RobotController(2)
    robot3 = RobotController(3)
    robot4 = RobotController(4)
    robot5 = RobotController(5)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot1)
    executor.add_node(robot2)
    executor.add_node(robot3)
    executor.add_node(robot4)
    executor.add_node(robot5)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot1.destroy_node()
        robot2.destroy_node()
        robot3.destroy_node()
        robot4.destroy_node()
        robot5.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
