# Exercice 4 - Surveillance avec planification jointe et théorie des jeux


## Disclaimer : Il y a un petit bug visuel à l'initialisation de minmax, mais ça ira après quelques secondes(En fait je n'ai pas bien réussi à clear les topics des exos précédents, dû au fait que j'ai copié collé les fichiers launch, etc des exercices précédents.


## Description
Ce programme implémente deux robots de surveillance qui doivent surveiller plusieurs points dans un environnement.
Deux méthodes sont utilisées pour résoudre ce problème:
1. Planification jointe: les robots coordonnent leurs actions pour maximiser l'utilité globale
2. Théorie des jeux (minimax): chaque robot optimise sa propre utilité en tenant compte des actions de l'autre

## Prérequis
- ROS 2
- Python 3
- Packages requis: nav2_map_server, nav2_lifecycle_manager, rviz2

## Installation
1. Placez ce dossier dans votre workspace ROS 2 (par exemple ~/ros2_ws/src/)
2. Compilez le package:

cd ~/ros2_ws
colcon build --packages-select exercice4

## Exécution
Pour lancer la simulation avec planification jointe:

ros2 launch exercice4 exercice4_joint.py

Pour lancer la simulation avec l'algorithme minimax:

ros2 launch exercice4 exercice4_minimax.py

## Fonctionnalités
- 2 robots de surveillance
- 6 points de surveillance générés aléatoirement
- Planification de chemin avec A*
- Deux modes de fonctionnement: planification jointe et minimax
- Visualisation dans RViz
