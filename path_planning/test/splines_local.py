import numpy as np
import matplotlib.pyplot as plt

def cubic_spline_coeffs(p0, v0, p1, v1, tf):
    T = np.array([
        [1,    0,      0,        0],
        [0,    1,      0,        0],
        [1,   tf,   tf**2,   tf**3],
        [0,    1,   2*tf,  3*tf**2]
    ])
    X = np.array([p0, v0, p1, v1])
    a = np.linalg.solve(T, X)
    return a  # [a0, a1, a2, a3]

def eval_cubic_spline(a, t):
    pos = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
    vel = a[1] + 2*a[2]*t + 3*a[3]*t**2
    return pos, vel

if __name__ == "__main__":
    path = [(0, 0), (1, 2), (2, 1), (3, 3), (4, 0)]
    tf = 2

    vx_points = [0]
    vy_points = [0]
    for i in range(1, len(path)-1):
        vx = 0.5 * ((path[i+1][0] - path[i-1][0]) / tf)
        vy = 0.5 * ((path[i+1][1] - path[i-1][1]) / tf)
        vx_points.append(vx)
        vy_points.append(vy)
    vx_points.append(0)
    vy_points.append(0)

    t_acum = 0
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i+1]
        v0x = vx_points[i]
        v1x = vx_points[i+1]
        v0y = vy_points[i]
        v1y = vy_points[i+1]
        ax = cubic_spline_coeffs(x0, v0x, x1, v1x, tf)
        ay = cubic_spline_coeffs(y0, v0y, y1, v1y, tf)
        t_segment = np.linspace(0, tf, 20)
        x_tramo, y_tramo, vx_tramo, vy_tramo, t_tramo = [], [], [], [], []
        for t in t_segment:
            px, vx = eval_cubic_spline(ax, t)
            py, vy = eval_cubic_spline(ay, t)
            x_tramo.append(px)
            y_tramo.append(py)
            vx_tramo.append(vx)
            vy_tramo.append(vy)
            t_tramo.append(t_acum + t)

        # Graficar solo el tramo actual
        plt.figure(figsize=(8,4))
        plt.subplot(1,2,1)
        plt.plot(x_tramo, y_tramo, color='blue', label='Spline tramo')
        plt.scatter(x_tramo, y_tramo, color='green', s=10, label='Puntos spline')
        plt.scatter(*zip(*path), color='red', zorder=5, label='Puntos de paso')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.axis('equal')
        plt.grid()
        plt.title(f'Tramo {i+1}')

        plt.subplot(1,2,2)
        plt.plot(t_tramo, vx_tramo, label='Vx')
        plt.plot(t_tramo, vy_tramo, label='Vy')
        plt.xlabel('Tiempo')
        plt.ylabel('Velocidad')
        plt.legend()
        plt.grid()
        plt.title(f'Velocidades tramo {i+1}')

        plt.tight_layout()
        plt.show()

        # Puedes pausar entre tramos si quieres simular el avance
        # input("Presiona Enter para continuar al siguiente tramo...")

        t_acum += tf