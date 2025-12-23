from exercice4.vec2d import Vec2D

class Map2D:
    def __init__(self, _map):
        self.map = _map

    # Étant donné un vecteur de coordonnées, renvoie la position (index) correspondante dans le tableau d'occupation de la carte
    # Attention à la résolution !
    def convert_coords(self, pos):
        x = pos.x
        y = pos.y
        width = self.map.info.width
        resolution = self.map.info.resolution

        # Vérifier si la résolution est valide pour éviter la division par zéro
        if resolution <= 0.0:
            resolution = 0.2  # Valeur par défaut
            print(f"WARNING: Map resolution was invalid, using default value of {resolution}")

        # Convertir les coordonnées du monde en indices de la grille
        x_index = int((x - self.map.info.origin.position.x) / resolution)
        y_index = int((y - self.map.info.origin.position.y) / resolution)

        # Calculer l'indice dans le tableau de données
        index = y_index * width + x_index
        return index

    # Étant donné un vecteur de coordonnées, renvoie un booléen selon l'état d'occupation de la position (True si la position est occupée, False sinon)
    def is_wall(self, pos):
        # Vérifier si les coordonnées sont dans les limites de la carte
        x = pos.x
        y = pos.y
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution

        # Vérifier si la résolution est valide pour éviter la division par zéro
        if resolution <= 0.0:
            resolution = 0.2  # Valeur par défaut
            print(f"WARNING: Map resolution was invalid, using default value of {resolution}")

        x_index = int((x - self.map.info.origin.position.x) / resolution)
        y_index = int((y - self.map.info.origin.position.y) / resolution)

        if x_index < 0 or x_index >= width or y_index < 0 or y_index >= height:
            return True  # Considérer les positions hors limites comme des murs

        index = self.convert_coords(pos)
        occupation = self.map.data[index]

        # Vérifier si la cellule est occupée (valeur > 0)
        return occupation > 0

    # Étant donné un point, vérifie si ce point est bien dans les limites de la carte
    # Renvoie False si le point est en dehors des limites, True sinon
    def in_bounds(self, pos):
        ## à compléter (retirer le pass)
        pass

    # Étant donné un point (vecteur) renvoie les points adjacents sur la carte (sous forme liste de vecteur)
    # On considère comme adjacent tout point dans les 8 directions autour du point donné
    # Les murs ne sont pas considérés comme des points adjacents
    # Faites attention aux bordures de la carte
    def get_adjacent(self, pos):
        ## à compléter (retirer le pass)
        pass

    # Étant donnés deux points (vecteurs), renvoie la distance euclidienne entre les deux points
    def euclidean_distance(self, a, b):
        ## à compléter (retirer le pass)
        pass
