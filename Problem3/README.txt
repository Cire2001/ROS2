# Exercice 3 - Convoi de robots avec algorithme A*

## Description
Ce programme implémente un convoi de robots dans un environnement avec obstacles.
Les robots utilisent l'algorithme A* pour planifier leurs trajets.
Le leader du convoi est déterminé comme étant le robot le plus proche de la destination,
et les autres robots suivent ce leader dans une formation en colonne.

## Prérequis
- ROS 2 Humble
- Python 3
- Packages requis: nav2_map_server, nav2_lifecycle_manager, rviz2

## Installation
1. Placez ce dossier dans votre workspace ROS 2 (par exemple ~/ros2_ws/src/)
2. Compilez le package:

cd ~/ros2_ws
colcon build --packages-select exercice3

## Exécution
Pour lancer la simulation:

ros2 launch exercice3 exercice3.py

Pour définir une destination pour le convoi:

ros2 topic pub -1 /set_destination geometry_msgs/msg/Point "{x: 50.0, y: 50.0, z: 0.0}"

## Fonctionnalités
- 5 robots formant un convoi organisé
- Sélection dynamique du leader basée sur la proximité à la destination
- Planification de chemins avec A*
- Suivi des robots en formation
- Évitement d'obstacles
- Visualisation dans RViz
