# Diferencias

## Para poder probar la detección de diferencias 2D, hay que lanzar las siguientes comandos en terminales diferentes:

$ python3 maquina_estados.py #La máquina se lanza y se queda en estado de espera hasta que se pulsa enter en dicho terminal.

$ ros2 launch mvsim launch_world.launch.py world_file:=world/new_world_modified.world.xml do_fake_localization:=false #Lanza el mundo simulado.

$ ros2 launch nav2_bringup bringup_launch.py map:=./world/mapa.yaml #Lanza el nodo para la navegación en el mapa generado. Se debe marcar la posición inicial del robot en el mapa de rviz.

$ python3 marker.py #Lanza el nodo para la detección de diferencias.

#Volver al terminal de maquina_estados.py y pulsar enter para empezar la navegación.
