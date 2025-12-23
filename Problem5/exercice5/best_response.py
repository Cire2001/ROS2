import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from nav_msgs.srv import GetMap
import numpy as np
import math
from exercice5.vec2d import Vec2D
from exercice5.map2d import Map2D
from exercice5.astar import AStar
from exercice5.stochastic_environment import StochasticEnvironment
import time

class BestResponse(Node):
    def __init__(self):
        super().__init__('best_response')
        self.get_logger().info("Initializing Best Response algorithm")

        # Paramètres de l'algorithme
        self.gamma = 0.95  # Facteur de discount
        self.epsilon = 0.01  # Seuil de convergence
        self.max_iterations = 50  # Réduit pour accélérer (était 100)

        # Nombre de robots
        self.num_robots = 3

        # Destination finale des robots
        self.destination = None

        # Positions actuelles des robots
        self.robot_positions = {}

        # Actions possibles pour chaque robot
        self.actions = ['north', 'south', 'east', 'west', 'wait']

        # Politiques des robots (dictionnaire robot_id -> (dictionnaire position -> action))
        self.robot_policies = {}

        # Cache pour les transitions
        self.transition_cache = {}

        # Cache pour les états proches
        self.nearest_state_cache = {}

        # Map et pathfinding
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

        # Initialiser l'environnement stochastique
        self.env = StochasticEnvironment(self.map)

        # États discrétisés pour le MDP
        self.states = []

        # Abonnements aux positions des robots
        for i in range(1, self.num_robots + 1):
            self.create_subscription(
                PoseStamped,
                f'/robot_{i}/current_position',
                lambda msg, robot_id=i: self.robot_position_callback(msg, robot_id),
                10)

        # Abonnement à la destination
        self.create_subscription(
            Point,
            '/set_destination',
            self.destination_callback,
            10)

        # Publication des politiques calculées
        self.policy_pub = self.create_publisher(
            String,
            '/best_response_policy',
            10)

        # Timer pour calculer les politiques
        self.create_timer(5.0, self.calculate_policies)

        self.get_logger().info("Best Response node initialized")

    def robot_position_callback(self, msg, robot_id):
        """Callback pour la position d'un robot"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_positions[robot_id] = Vec2D(x, y)

    def destination_callback(self, msg):
        """Callback pour la destination"""
        self.destination = Vec2D(msg.x, msg.y)
        self.get_logger().info(f"Destination set to: ({msg.x}, {msg.y})")

        # Discrétiser l'espace d'états quand on reçoit la destination
        self.discretize_state_space()

        # Vider les caches quand la destination change
        self.transition_cache = {}
        self.nearest_state_cache = {}

    def discretize_state_space(self):
        """Discrétiser l'espace d'états avec une grille adaptative"""
        self.states = []

        # Si nous n'avons pas encore la destination ou les positions des robots, utiliser une grille régulière
        if not self.destination or len(self.robot_positions) < self.num_robots:
            grid_size = 5  # Taille de la cellule de la grille
            for x in range(0, 100, grid_size):
                for y in range(0, 100, grid_size):
                    pos = Vec2D(float(x), float(y))
                    if not self.map.is_wall(pos):
                        self.states.append(pos)
        else:
            # Utiliser une grille adaptative avec une résolution plus fine près des positions des robots
            important_points = list(self.robot_positions.values()) + [self.destination]

            # Grille grossière partout
            grid_size_coarse = 10
            for x in range(0, 100, grid_size_coarse):
                for y in range(0, 100, grid_size_coarse):
                    pos = Vec2D(float(x), float(y))
                    if not self.map.is_wall(pos):
                        self.states.append(pos)

            # Grille fine autour des points importants
            grid_size_fine = 2
            radius = 20  # Rayon autour des points importants pour une grille fine

            for point in important_points:
                min_x = max(0, int(point.x - radius))
                max_x = min(100, int(point.x + radius))
                min_y = max(0, int(point.y - radius))
                max_y = min(100, int(point.y + radius))

                for x in range(min_x, max_x, grid_size_fine):
                    for y in range(min_y, max_y, grid_size_fine):
                        pos = Vec2D(float(x), float(y))
                        if not self.map.is_wall(pos) and not self._state_exists(pos):
                            self.states.append(pos)

        # Limiter le nombre d'états pour des performances raisonnables
        if len(self.states) > 500:
            # Garder les états proches des robots et de la destination
            important_states = []
            for state in self.states:
                # Vérifier si l'état est proche d'un point important
                for point in important_points:
                    if state.distance_to(point) < 30:
                        important_states.append(state)
                        break

            # Si nous avons encore trop d'états, échantillonner
            if len(important_states) > 200:
                # Échantillonner aléatoirement
                indices = np.random.choice(len(important_states), 200, replace=False)
                self.states = [important_states[i] for i in indices]
            else:
                self.states = important_states

        self.get_logger().info(f"Created state space with {len(self.states)} states")

    def _state_exists(self, pos, threshold=1.0):
        """Vérifier si un état similaire existe déjà dans la liste des états"""
        for state in self.states:
            if state.distance_to(pos) < threshold:
                return True
        return False

    def calculate_policies(self):
        """Calculer les politiques des robots avec l'algorithme Best Response"""
        if not self.destination or len(self.robot_positions) < self.num_robots:
            # Destination ou positions des robots non encore définies
            return

        self.get_logger().info("Calculating policies using Best Response algorithm")

        # Initialiser les politiques aléatoirement si elles n'existent pas
        if not self.robot_policies:
            self.initialize_random_policies()

        # Exécuter l'algorithme Best Response
        for i in range(10):  # Nombre d'itérations global pour Best Response
            self.get_logger().info(f"Best Response iteration {i+1}/10")

            policy_changed = False

            # Pour chaque robot, améliorer sa politique en réponse aux politiques des autres
            for robot_id in range(1, self.num_robots + 1):
                self.get_logger().info(f"Computing best response for robot {robot_id}")
                other_policies = self.get_other_robot_policies(robot_id)
                new_policy = self.compute_best_response(robot_id, other_policies)

                # Vérifier si la politique a changé
                if self.policy_has_changed(self.robot_policies[robot_id], new_policy):
                    self.robot_policies[robot_id] = new_policy
                    policy_changed = True
                    self.get_logger().info(f"Policy for robot {robot_id} has been updated")

            # Si aucune politique n'a changé, sortir de la boucle
            if not policy_changed:
                self.get_logger().info(f"Convergence reached at iteration {i+1}")
                break

        # Publier les politiques calculées
        self.publish_policies()

    def initialize_random_policies(self):
        """Initialiser des politiques aléatoires pour tous les robots"""
        for robot_id in range(1, self.num_robots + 1):
            policy = {}

            # Utiliser les états discrétisés
            for state in self.states:
                # Choisir une action aléatoire
                action = np.random.choice(self.actions)
                policy[state] = action

            self.robot_policies[robot_id] = policy

        self.get_logger().info(f"Initialized random policies for {self.num_robots} robots")

    def get_other_robot_policies(self, robot_id):
        """Obtenir les politiques des autres robots"""
        other_policies = {}
        for other_id, policy in self.robot_policies.items():
            if other_id != robot_id:
                other_policies[other_id] = policy
        return other_policies

    def policy_has_changed(self, old_policy, new_policy):
        """Vérifier si la politique a changé"""
        if len(old_policy) != len(new_policy):
            return True

        # Vérifier un échantillon d'états pour accélérer la comparaison
        sample_size = min(30, len(old_policy))
        sample_states = np.random.choice(list(old_policy.keys()), sample_size, replace=False)

        for state in sample_states:
            if state not in new_policy or new_policy[state] != old_policy[state]:
                return True

        return False

    def compute_best_response(self, robot_id, other_policies):
        """Calculer la meilleure réponse aux politiques des autres robots"""
        start_time = time.time()

        # Sélectionner les états pertinents pour ce robot
        relevant_states = self._get_relevant_states(robot_id)

        # Initialiser les valeurs V pour les états pertinents
        # Utiliser l'heuristique QMDP pour meilleure initialisation (p.80 du cours)
        V = self._initialize_values_with_heuristic(relevant_states, robot_id)

        # Itération de valeur pour le MDP augmenté - Implémentation de la page 83-84 du cours
        for iteration in range(self.max_iterations):
            max_diff = 0.0

            for state in relevant_states:
                old_value = V.get(state, 0.0)

                # Calculer la valeur maximale sur toutes les actions
                max_value = float('-inf')

                for action in self.actions:
                    # Obtenir les actions des autres agents selon leurs politiques
                    other_actions = self.get_other_actions(state, other_policies)

                    # Calculer Q((s1,s2), (a, π(s2)))
                    q_value = self._calculate_q_value(state, action, other_actions, V)

                    if q_value > max_value:
                        max_value = q_value

                # Mettre à jour V(s1,s2)
                V[state] = max_value
                max_diff = max(max_diff, abs(V[state] - old_value))

            # Vérifier la convergence
            if max_diff < self.epsilon:
                elapsed = time.time() - start_time
                self.get_logger().info(f"VI converged after {iteration+1} iterations in {elapsed:.2f}s")
                break

        # Extraire la politique optimale
        policy = {}
        for state in relevant_states:
            best_action = None
            best_value = float('-inf')

            for action in self.actions:
                other_actions = self.get_other_actions(state, other_policies)
                q_value = self._calculate_q_value(state, action, other_actions, V)

                if q_value > best_value:
                    best_value = q_value
                    best_action = action

            policy[state] = best_action

        return policy

    def _get_relevant_states(self, robot_id):
        """Obtenir les états pertinents pour un robot donné"""
        if not self.robot_positions or robot_id not in self.robot_positions:
            return self.states

        relevant_states = []
        robot_pos = self.robot_positions[robot_id]

        # Distance jusqu'à la destination
        dist_to_dest = robot_pos.distance_to(self.destination)

        # Les états pertinents sont ceux qui sont entre le robot et la destination,
        # plus une marge pour tenir compte des détours possibles
        for state in self.states:
            # Si l'état est proche du robot ou de la destination, c'est pertinent
            if (state.distance_to(robot_pos) < dist_to_dest * 0.8 or
                state.distance_to(self.destination) < dist_to_dest * 0.8):
                relevant_states.append(state)

        # Si nous avons trop peu d'états, utiliser tous les états
        if len(relevant_states) < 50:
            return self.states

        return relevant_states

    def _initialize_values_with_heuristic(self, states, robot_id):
        """Initialiser les valeurs selon l'heuristique QMDP (p.80 du cours)"""
        values = {}

        # Pour chaque état, utiliser -distance comme valeur initiale
        for state in states:
            # Valeur basée sur la distance à la destination (heuristique simple)
            values[state] = -state.distance_to(self.destination)

        return values

    def _calculate_q_value(self, state, action, other_actions, V):
        """Calculer la valeur Q pour un état et une action"""
        q_value = 0.0

        # Obtenir les transitions possibles (avec cache)
        next_states_probs = self.get_transitions(state, action, other_actions)

        for next_state, prob in next_states_probs.items():
            reward = self.calculate_reward(state, next_state)
            # Utiliser 0.0 comme valeur par défaut pour les états non visités
            next_value = V.get(next_state, 0.0)
            q_value += prob * (reward + self.gamma * next_value)

        return q_value

    def get_other_actions(self, state, other_policies):
        """Obtenir les actions des autres agents pour un état donné"""
        other_actions = {}

        for other_id, policy in other_policies.items():
            # Utiliser le cache pour trouver l'état le plus proche
            nearest_state = self._find_nearest_state_in_policy(state, policy)

            if nearest_state:
                other_actions[other_id] = policy[nearest_state]
            else:
                other_actions[other_id] = 'wait'  # Action par défaut

        return other_actions

    def _find_nearest_state_in_policy(self, state, policy):
        """Trouver l'état le plus proche dans une politique (avec cache)"""
        # Générer une clé de cache
        cache_key = (state, id(policy))

        # Vérifier si nous avons déjà calculé cet état
        if cache_key in self.nearest_state_cache:
            return self.nearest_state_cache[cache_key]

        # Sinon, le calculer
        nearest_state = None
        min_distance = float('inf')

        for policy_state in policy.keys():
            distance = state.distance_to(policy_state)
            if distance < min_distance:
                min_distance = distance
                nearest_state = policy_state

        # Mettre en cache le résultat
        self.nearest_state_cache[cache_key] = nearest_state

        return nearest_state

    def get_transitions(self, state, action, other_actions):
        """Obtenir les transitions possibles et leurs probabilités avec cache"""
        # Générer une clé de cache
        cache_key = (state, action, frozenset(other_actions.items()))

        # Vérifier si nous avons déjà calculé cette transition
        if cache_key in self.transition_cache:
            return self.transition_cache[cache_key]

        # Convertir l'action en format de l'environnement stochastique
        action_map = {
            'north': 'up',
            'south': 'down',
            'east': 'right',
            'west': 'left',
            'wait': 'wait'
        }
        env_action = action_map.get(action, 'wait')

        # Si l'action est 'wait', rester sur place avec certitude
        if env_action == 'wait':
            result = {state: 1.0}
            self.transition_cache[cache_key] = result
            return result

        # Sinon, utiliser l'environnement stochastique
        try:
            probs = self.env.get_transition_probabilities(state, env_action)
        except ValueError:
            # Si l'action n'est pas reconnue, rester sur place
            result = {state: 1.0}
            self.transition_cache[cache_key] = result
            return result

        # Filtre pour garder seulement les états valides
        filtered_probs = {}
        for next_state, prob in probs.items():
            # Trouver l'état discret le plus proche
            closest_state = self.find_closest_state(next_state)

            if closest_state in filtered_probs:
                filtered_probs[closest_state] += prob
            else:
                filtered_probs[closest_state] = prob

        # Mettre en cache le résultat
        self.transition_cache[cache_key] = filtered_probs

        return filtered_probs

    def find_closest_state(self, position):
        """Trouver l'état discret le plus proche d'une position continue"""
        # Vérifier le cache
        if position in self.nearest_state_cache:
            return self.nearest_state_cache[position]

        closest_state = None
        min_distance = float('inf')

        for state in self.states:
            distance = position.distance_to(state)
            if distance < min_distance:
                min_distance = distance
                closest_state = state

        # Mettre en cache le résultat
        self.nearest_state_cache[position] = closest_state or position

        return closest_state or position  # Fallback à la position d'origine

    def calculate_reward(self, state, next_state):
        """Calculer la récompense pour une transition d'état"""
        # Distance à la destination (récompense négative car on veut minimiser)
        distance_reward = -next_state.distance_to(self.destination)

        # Pénalité si on est dans un mur
        if self.map.is_wall(next_state):
            return -1000.0

        # Bonus si on atteint la destination
        if next_state.distance_to(self.destination) < 2.0:
            return 100.0

        return distance_reward

    def get_next_position(self, pos, action):
        """Obtenir la position suivante après une action"""
        next_pos = Vec2D(pos.x, pos.y)

        if action == 'north' or action == 'up':
            next_pos.y += 1.0
        elif action == 'south' or action == 'down':
            next_pos.y -= 1.0
        elif action == 'east' or action == 'right':
            next_pos.x += 1.0
        elif action == 'west' or action == 'left':
            next_pos.x -= 1.0
        # 'wait' ne change pas la position

        # Vérifier si la position suivante est un mur
        if self.map.is_wall(next_pos):
            return pos

        return next_pos

    def publish_policies(self):
        """Publier les politiques calculées"""
        policy_msg = String()
        policy_str = "Best Response Policies:\n"

        for robot_id, policy in self.robot_policies.items():
            policy_str += f"Robot {robot_id}:\n"

            # Afficher quelques actions pour chaque position (limité pour éviter les messages trop longs)
            count = 0
            for pos, action in policy.items():
                policy_str += f"  Position ({pos.x}, {pos.y}): {action}\n"
                count += 1
                if count >= 10:
                    break

            policy_str += f"  Total positions: {len(policy)}\n"

        policy_msg.data = policy_str
        self.policy_pub.publish(policy_msg)
        self.get_logger().info("Published Best Response policies")

def main(args=None):
    rclpy.init(args=args)
    node = BestResponse()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
