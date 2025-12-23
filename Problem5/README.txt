# Exercice 5 - Convoi de robots avec algorithmes Best_Response et JESP

## Description
Ce programme implémente un convoi de robots dans un environnement stochastique avec obstacles.
Les robots utilisent les algorithmes Best_Response et JESP pour calculer leur politique jointe.
Le leader du convoi est déterminé comme étant le robot le plus proche de la destination.

## Prérequis
- ROS 2 
- Python 3
- Packages requis: nav2_map_server, nav2_lifecycle_manager, rviz2

## Installation
1. Placez ce dossier dans votre workspace ROS 2 (par exemple ~/ros2_ws/src/)
2. Compilez le package:

cd ~/ros2_ws
colcon build --packages-select exercice5



## Exécution
Pour lancer la simulation:

ros2 launch exercice5 exercice5.py


Pour définir une destination pour le convoi, il faut cependant donner une destination valide:

ros2 topic pub -1 /set_destination geometry_msgs/msg/Point "{x: 70.0, y: 70.0, z: 0.0}"


## Fonctionnalités
- 3 robots formant un convoi
- Environnement stochastique (probabilités 0.8, 0.1, 0.1)
- Calcul de politique avec Best_Response et JESP
- Visualisation dans RViz
