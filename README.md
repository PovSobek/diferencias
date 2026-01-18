# Diferencias

### Para poder probar la detección de diferencias 2D, hay que lanzar las siguientes comandos en terminales diferentes:

$ python3 maquina_estados.py *#La máquina se lanza y se queda en estado de espera hasta que se pulsa enter en dicho terminal.*

$ ros2 launch mvsim launch_world.launch.py world_file:=world/new_world_modified.world.xml do_fake_localization:=false *#Lanza el mundo simulado.*

$ ros2 launch nav2_bringup bringup_launch.py map:=./world/mapa.yaml *#Lanza el nodo para la navegación en el mapa generado. Se debe marcar la posición inicial del robot en el mapa de rviz.*

$ python3 marker.py *#Lanza el nodo para la detección de diferencias.*

*#Volver al terminal de maquina_estados.py y pulsar enter para empezar la navegación.*


### Para poder probar la detección de diferencias 3D:

$ python3 maquina_estados.py *#La máquina se lanza y se queda en estado de espera hasta que se pulsa enter en dicho terminal.*


$ ros2 launch mvsim launch_world.launch.py world_file:=world/new_world_modified.world.xml do_fake_localization:=false *#Lanza el mundo simulado.*


$ python3 Nodo_nube3D.py *#Lanza el nodo para realizar un mapa de nubes de puntos 3D sobre el mundo actual*


$ ros2 launch nav2_bringup bringup_launch.py map:=./world/mapa.yaml *#Lanza el nodo para la navegación en el mapa generado. Se debe marcar la posición inicial del robot en el mapa de rviz.*


*#Volver al terminal de maquina_estados.py y pulsar enter para empezar la navegación.
#Una vez terminada la navegación se presiona Ctrl+C para terminar todas las ejecuciones, guardando el mapa de nubes de puntos en un archivo .pcd.
#Se asignan los nombres en el archivo Nodo_comparar_nubes.py de las nubes de puntos.*

$ python3 Nodo_ comparar_nubes.py *#Lanza el nodo para realizar una imagen con la comparación de los dos mapas de nubes de puntos, mostrando las diferencias.*
