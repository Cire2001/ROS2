# astar.py
import heapq
from exercice5.vec2d import Vec2D  # Changé de exercice1.vec2d à exercice3.vec2d
import math

class AStar:
    def __init__(self, map_2d):
        self.map = map_2d

    def heuristic(self, a, b):
        # Distance euclidienne
        return a.distance_to(b)

    def get_neighbors(self, node):
        # Retourne les voisins valides (non murs) d'un nœud
        neighbors = []
        directions = [
            Vec2D(1, 0),   # Droite
            Vec2D(-1, 0),  # Gauche
            Vec2D(0, 1),   # Haut
            Vec2D(0, -1),  # Bas
            Vec2D(1, 1),   # Diagonale haut-droite
            Vec2D(-1, 1),  # Diagonale haut-gauche
            Vec2D(1, -1),  # Diagonale bas-droite
            Vec2D(-1, -1)  # Diagonale bas-gauche
        ]

        for direction in directions:
            neighbor = node + direction
            if not self.map.is_wall(neighbor):
                neighbors.append(neighbor)

        return neighbors

    def reconstruct_path(self, came_from, current):
        # Reconstruit le chemin à partir de la carte des prédécesseurs
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.insert(0, current)
        return total_path

    def find_path(self, start, goal):
        # Implémentation de l'algorithme A*
        if self.map.is_wall(start) or self.map.is_wall(goal):
            return []

        # Ensemble des nœuds à explorer
        open_set = []
        # Utiliser un tuple (f_score, id(node), node) pour éviter la comparaison directe des Vec2D
        heapq.heappush(open_set, (0, id(start), start))

        # Carte des prédécesseurs pour reconstruire le chemin
        came_from = {}

        # Coût du chemin du départ à chaque nœud
        g_score = {start: 0}

        # Estimation du coût total du départ au but en passant par chaque nœud
        f_score = {start: self.heuristic(start, goal)}

        # Ensemble des nœuds déjà visités
        closed_set = set()

        while open_set:
            # Récupérer le nœud avec le score f le plus bas
            _, _, current = heapq.heappop(open_set)

            # Si nous avons atteint le but, reconstruire et retourner le chemin
            if current.distance_to(goal) < 1.0:
                return self.reconstruct_path(came_from, current)

            # Marquer le nœud comme visité
            closed_set.add(current)

            # Explorer les voisins
            for neighbor in self.get_neighbors(current):
                # Ignorer les voisins déjà visités
                if neighbor in closed_set:
                    continue

                # Calculer le coût tentative pour atteindre ce voisin
                tentative_g_score = g_score.get(current, float('inf')) + current.distance_to(neighbor)

                # Si ce chemin vers le voisin est meilleur que tout chemin précédent, le mettre à jour
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # Ce chemin est le meilleur jusqu'à présent, l'enregistrer
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)

                    # Ajouter le voisin à l'ensemble des nœuds à explorer s'il n'y est pas déjà
                    if all(neighbor != node for _, _, node in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], id(neighbor), neighbor))

        # Aucun chemin trouvé
        return []
