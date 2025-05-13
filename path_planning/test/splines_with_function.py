import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# 1. Definir waypoints (puntos de control)
waypoints = np.array([
    [0, 0],     # Punto inicial
    [2, 3],     # Primer waypoint
    [5, 2],     # Segundo waypoint
    [8, 5],     # Tercer waypoint
    [10, 0]     # Punto final
])

# 2. Crear trayectoria poligonal (normal)
x_poly = waypoints[:, 0]
y_poly = waypoints[:, 1]

# 3. Crear spline cúbico parametrizado
t = np.arange(len(waypoints))  # Parámetro basado en índices
cs_x = CubicSpline(t, x_poly, bc_type='natural')  # Spline para X
cs_y = CubicSpline(t, y_poly, bc_type='natural')  # Spline para Y

# 4. Generar puntos suavizados
t_fine = np.linspace(0, len(waypoints)-1, 100)  # 100 puntos suavizados
x_spline = cs_x(t_fine)
y_spline = cs_y(t_fine)

# 5. Graficar comparación
plt.figure(figsize=(10, 6))
plt.plot(x_poly, y_poly, 'ro--', label='Trayectoria Poligonal', linewidth=2)
plt.plot(x_spline, y_spline, 'b-', label='Trayectoria Spline', linewidth=2)
plt.scatter(x_poly, y_poly, c='red', s=100, zorder=3)  # Destacar waypoints

# Configuración del gráfico
plt.title('Comparación de Trayectorias', fontsize=14)
plt.xlabel('Coordenada X', fontsize=12)
plt.ylabel('Coordenada Y', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.axis('equal')
plt.show()