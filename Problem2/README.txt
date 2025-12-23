# Exercice 2 - Rendez-vous de robots

## Description
Ce programme implémente un algorithme de rendez-vous entre robots dans un environnement avec obstacles.
Les robots naviguent en utilisant l'algorithme A* et se rencontrent à mi-chemin.

## Prérequis
- ROS 2 
- Python 3
- Packages requis: nav2_map_server, nav2_lifecycle_manager, rviz2

## Installation
1. Placez ce dossier dans votre workspace ROS 2 (par exemple ~/ros2_ws/src/)
2. Compilez le package:

cd ~/ros2_ws
colcon build --packages-select exercice2

## Exécution
Pour lancer la simulation:

ros2 launch exercice2 exercice2.py

Pour déclencher le rendez-vous entre les robots:

ros2 topic pub -1 /trigger_rendezvous std_msgs/msg/String "data: 'start'"

## Fonctionnalités
- 3 robots avec positions initiales différentes
- Navigation avec A* évitant les obstacles
- Algorithme de rendez-vous
- Visualisation dans RViz
