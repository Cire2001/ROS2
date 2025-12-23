# exercice5/stochastic_environment.py
import numpy as np
from exercice5.vec2d import Vec2D
from exercice5.map2d import Map2D

class StochasticEnvironment:
    def __init__(self, map_2d):
        self.map = map_2d
        self.grid_size = 10  # Taille de la grille 10x10 pour l'exercice 5

    def get_transition_probabilities(self, position, action):
        """
        Calcule les probabilités de transition pour une position et une action données.

        Args:
            position (Vec2D): Position actuelle
            action (str): Action à effectuer ('up', 'down', 'left', 'right')

        Returns:
            dict: Dictionnaire {position_résultante: probabilité}
        """
        # Définir les déplacements pour chaque action
        action_vectors = {
            'up': Vec2D(0, 1),
            'down': Vec2D(0, -1),
            'left': Vec2D(-1, 0),
            'right': Vec2D(1, 0)
        }

        # Vérifier si l'action est valide
        if action not in action_vectors:
            raise ValueError(f"Action {action} non reconnue")

        # Calculer la position cible principale
        target_position = position + action_vectors[action]

        # Si la position cible est un mur, rester à la position actuelle
        if self.map.is_wall(target_position):
            target_position = position

        # Trouver les positions voisines possibles (perpendiculaires à l'action)
        perpendicular_actions = []
        if action in ['up', 'down']:
            perpendicular_actions = ['left', 'right']
        else:
            perpendicular_actions = ['up', 'down']

        # Calculer les positions voisines
        neighbor_positions = []
        for perp_action in perpendicular_actions:
            neighbor_pos = position + action_vectors[perp_action]
            if not self.map.is_wall(neighbor_pos):
                neighbor_positions.append(neighbor_pos)

        # Calculer les probabilités selon les règles de l'exercice 5
        probabilities = {}

        if len(neighbor_positions) == 2:
            # Cas avec deux voisins: (0.8, 0.1, 0.1)
            probabilities[target_position] = 0.8
            probabilities[neighbor_positions[0]] = 0.1
            probabilities[neighbor_positions[1]] = 0.1
        elif len(neighbor_positions) == 1:
            # Cas avec un seul voisin: (0.8, 0.2)
            probabilities[target_position] = 0.8
            probabilities[neighbor_positions[0]] = 0.2
        else:
            # Cas sans voisin: 1.0 pour la position cible
            probabilities[target_position] = 1.0

        return probabilities

    def sample_next_state(self, position, action):
        """
        Échantillonne l'état suivant en fonction des probabilités de transition.

        Args:
            position (Vec2D): Position actuelle
            action (str): Action à effectuer

        Returns:
            Vec2D: Nouvelle position
        """
        probs = self.get_transition_probabilities(position, action)
        positions = list(probs.keys())
        probabilities = list(probs.values())

        # Échantillonner la prochaine position
        next_position = np.random.choice(positions, p=probabilities)
        return next_position

def main(args=None):
    # Ce nœud n'est pas destiné à être exécuté seul
    pass

if __name__ == '__main__':
    main()
