### 1. Carga el mapa
    Lee el .pgm y .yaml para obtener la matriz de ocupación.

### 2. Recibe el objetivo
    Puedes suscribirte a un tópico (por ejemplo, /goal_pose) o definirlo manualmente.

### 3. Obtén la posición actual del robot
    Usa la odometría o el tf (/odom o /tf).

### 4. Convierte coordenadas reales a índices de la matriz
    Usa la resolución y el origen del mapa.

### 5. Ejecuta tu algoritmo A*
    Usa tu función a_star(start, goal, grid).

### 6. Convierte la ruta de índices a coordenadas reales
    Para enviar comandos de movimiento.

### 7. Publica la ruta o comandos de movimiento
    Puedes publicar la ruta como nav_msgs/Path o enviar comandos de velocidad (geometry_msgs/Twist).