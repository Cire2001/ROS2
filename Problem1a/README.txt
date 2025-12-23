README - Exercice 1
Description

Ce programme permet à plusieurs robots dans un environnement de se diriger vers une même destination commune. Aucun robot n'occupe exactement la destination, mais chacun se place à la position libre la plus proche autour de celle-ci.
Prérequis

    ROS 2
    Python 3
    Packages requis: nav2_map_server, nav2_lifecycle_manager, rviz2

Installation

    Placez ce dossier dans votre workspace ROS 2 (par exemple ~/ros2_ws/src/)
    Compilez le package:

bash

Copy Code
cd ~/ros2_ws
colcon build --packages-select exercice1a
source install/setup.bash

Exécution

Pour lancer la simulation:

bash

Copy Code
ros2 launch exercice1a exercice1a.py

Ensuite, dans RViz:

    Utilisez l'outil "Publish Point" (raccourci 'P')
    Cliquez sur la carte pour définir une destination
    Observez les robots se déplacer vers des positions autour de la destination

Alternativement, vous pouvez définir une destination par commande:

bash

Copy Code
ros2 topic pub -1 /set_destination geometry_msgs/msg/Point "{x: 50.0, y: 50.0, z: 0.0}"

Fonctionnalités

    Gestion de 3 robots simultanément
    Planification de trajectoire avec A*
    Positionnement intelligent autour d'une destination commune
    Évitement des obstacles
    Visualisation dans RViz avec des marqueurs colorés pour chaque robot

Comportement attendu

Les robots se déplacent vers des positions disposées en cercle autour de la destination commune. Le rayon de ce cercle est configurable dans le code (valeur par défaut: 2.0 unités).
