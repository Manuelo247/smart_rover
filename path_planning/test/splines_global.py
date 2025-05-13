import numpy as np
import matplotlib.pyplot as plt

# =============================================
# 1. Definir puntos de control (waypoints)
# =============================================
points = np.array([
    [0, 0],    # P0
    [2, 3],    # P1
    [5, 2],    # P2
    [8, 5]     # P3
])

# =============================================
# 2. Calcular splines cúbicos manualmente
# =============================================
def compute_spline_coefficients(points_1d):
    n = len(points_1d)
    h = np.diff(np.arange(n))  # Asumimos t uniforme: h=1
    al = np.zeros(n)
    for i in range(1, n-1):
        al[i] = 3*(points_1d[i+1] - points_1d[i]) - 3*(points_1d[i] - points_1d[i-1])
    l = np.ones(n)
    mu = np.zeros(n)
    z = np.zeros(n)
    for i in range(1, n-1):
        l[i] = 4 - mu[i-1]
        mu[i] = 1 / l[i]
        z[i] = (al[i] - z[i-1]) / l[i]
    c = np.zeros(n)
    b = np.zeros(n-1)
    d = np.zeros(n-1)
    for j in range(n-2, -1, -1):
        c[j] = z[j] - mu[j]*c[j+1]
        b[j] = (points_1d[j+1] - points_1d[j]) - (c[j+1] + 2*c[j])/3
        d[j] = (c[j+1] - c[j]) / 3
    coeffs = []
    for i in range(n-1):
        coeffs.append([points_1d[i], b[i], c[i], d[i]])
    return coeffs

# Usar la función para X e Y
coeffs_x = compute_spline_coefficients(points[:, 0])
coeffs_y = compute_spline_coefficients(points[:, 1])

# =============================================
# 3. Generar trayectorias
# =============================================
def evaluate_spline(coeffs, t_segment, t):
    """Evalúa el spline en el tiempo t para un segmento dado."""
    a0, a1, a2, a3 = coeffs
    return a0 + a1*t + a2*t**2 + a3*t**3

# Tiempo por segmento (0 a 1 para cada segmento)
t_fine = []
for segment in range(len(points)-1):
    t_fine.extend(np.linspace(segment, segment+1, 25))
t_fine = np.array(t_fine)

x_spline, y_spline = [], []
for t in t_fine:
    segment = int(t)
    t_local = t - segment
    if segment >= len(coeffs_x):
        segment = len(coeffs_x)-1
        t_local = 1.0
    
    # Evalúa el polinomio cúbico
    x = coeffs_x[segment][0] + coeffs_x[segment][1]*t_local + coeffs_x[segment][2]*t_local**2 + coeffs_x[segment][3]*t_local**3
    y = coeffs_y[segment][0] + coeffs_y[segment][1]*t_local + coeffs_y[segment][2]*t_local**2 + coeffs_y[segment][3]*t_local**3
    x_spline.append(x)
    y_spline.append(y)

# Trayectoria recta (conexión directa entre puntos)
x_poly = points[:, 0]
y_poly = points[:, 1]

# =============================================
# 4. Graficar resultados
# =============================================
plt.figure(figsize=(10, 6))
plt.plot(x_poly, y_poly, 'ro--', label='Trayectoria Poligonal', linewidth=2)
plt.plot(x_spline, y_spline, 'b-', label='Trayectoria Spline', linewidth=2)
plt.scatter(x_poly, y_poly, c='red', s=100, zorder=3)

# Ajustes del gráfico
plt.title('Comparación: Trayectoria Poligonal vs Spline Cúbico', fontsize=14)
plt.xlabel('X', fontsize=12)
plt.ylabel('Y', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()
plt.axis('equal')
plt.show()