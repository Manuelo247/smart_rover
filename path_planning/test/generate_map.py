import numpy as np
import cv2
import os

# Configuración del mapa (resolución 0.05 m/píxel)
width, height = 400, 400  # 20x20 metros
resolution = 0.05

# Crear matriz (0 = libre, 100 = ocupado)
grid = np.zeros((height, width), dtype=np.uint8)

# Bordes y obstáculos
grid[0:height, 0] = 100          # Borde izquierdo
grid[0:height, width-1] = 100    # Borde derecho
grid[0, 0:width] = 100           # Borde superior
grid[height-1, 0:width] = 100    # Borde inferior

# Obstáculos personalizados (¡modifica aquí!)
grid[50:350, 50:100] = 100       # Pared vertical
grid[150:250, 200:300] = 100     # Cuadrado central
grid[300:350, 150:300] = 100     # Rectángulo inferior

# Ruta al directorio resource
resource_dir = os.path.join(os.path.dirname(__file__), '..', 'resource')
resource_dir = os.path.abspath(resource_dir)

# Asegúrate de que el directorio existe
os.makedirs(resource_dir, exist_ok=True)

# Guardar como .pgm
pgm_path = os.path.join(resource_dir, "ros2_custom_map.pgm")
cv2.imwrite(pgm_path, grid)

# Crear .yaml (ROS 2 usa el mismo formato)
yaml_path = os.path.join(resource_dir, "ros2_custom_map.yaml")
with open(yaml_path, "w") as f:
    f.write(f"""image: ros2_custom_map.pgm
resolution: {resolution}
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")