# Planificación de rutas para robot diferencial en ROS 2

## Algoritmos utilizados:
- **Theta\***: Algoritmo de planificación de rutas basado en A*, pero permite trayectorias más directas usando línea de visión.
- **A\***: Algoritmo clásico de búsqueda de caminos óptimos en grafos. (Sustituto)
- **Inflado de obstáculos**: Se utiliza para asegurar que el robot no planee rutas por zonas donde físicamente no cabe.
- **Costmap ponderado**: Cada celda libre tiene un coste adicional que depende de su cercanía a obstáculos, penalizando rutas cercanas a paredes.

---

## Flujo de código (resumido):

1. **Carga el mapa** desde archivos `.pgm` y `.yaml`.
2. **Infla los obstáculos** según el radio del robot para asegurar zonas seguras.
3. **Genera el costmap**: penaliza celdas cercanas a obstáculos usando la función de coste.
4. **Recibe la posición actual y el objetivo** mediante tópicos ROS 2.
5. **Valida el objetivo**: debe estar en zona segura y dentro del mapa.
6. **Planifica la ruta** usando Theta* (o A*) minimizando el coste acumulado.
7. **Publica la ruta** como `nav_msgs/Path` para visualización o control.

---