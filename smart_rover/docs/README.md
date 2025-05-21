# Testing

## Rover

### Variables de entorno importantes
El robot tiene que tener las siguientes variables de entorno para funciona. Agregalas al .bashrc para hacerlo permanente.
```bash
export ROS_DOMAIN_ID=32
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Correr codigos para controlar al robot
```bash
ros2 launch smart_rover init_rover.launch.py
```
#### Componentes del launch
El launch se compone de otros dos launch principales **external_control.launch.py** y **components.launch.py**.
- **external_control.launch.py**
```
ros2 run smart_rover angle # Controlador de angulo
ros2 run smart_rover linearization # Controlador de cinematica
ros2 run smart_rover path_follower # Seguidor de caminor
ros2 run smart_rover switch_controller # Controlador que cambia entre angle y linearization
ros2 run path_planning planning # Planificador de rutas
ros2 run path_planning splines # Metodo splines a rutas
```
- **components.launch.py**
```
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py  # Ejecuta la odometria y control del robot
ros2 launch oradar_lidar ms200_scan.launch.py # Inicializa el lidar
ros2 launch slam_gmapping slam_gmapping.launch.py # Ejecuta nodos para el slam (Odometria y Lidar incluidos)
```
## Maquina local
### Visualizacion
Para poder visualizar los topicos del robot tienes que tener el mismo ROS DOMAIN y MIDDLEWARE, ademas de estar conectados en la misma red del robot.
Recuerda exportar las variables de entorno con los siguientes valores:
```bash
export ROS_DOMAIN_ID=32
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
Para ejecutar rviz2 y visualizar los topicos del rover puedes abrirlo con:
```bash
rviz2 -d ~/8vo/ros2_ws/src/path_planning/rviz/path_planning.rviz  &
```