import sys, os
import cv2
import numpy as np
import threading
import time

# Importar librerías del proyecto
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from pyrealsense.RealSenseHandler import RealSenseHandler
from middleware.publisher.publisher import PublisherNode
from middleware.registry import load_msg

def show_camera():
    # Mostrar datos del acelerómetro
    if accel:
        print(f"Acelerómetro: x={accel[0]:.2f}, y={accel[1]:.2f}, z={accel[2]:.2f}")

    # Mostrar datos del giroscopio
    if gyro:
        print(f"Giroscopio: x={gyro[0]:.2f}, y={gyro[1]:.2f}, z={gyro[2]:.2f}")

    # Mostrar streams de profundidad y color
    if depth_image is not None and color_image is not None:
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        combined_image = np.hstack((color_image, depth_colormap))
        cv2.imshow('RealSense Stream (Color + Depth)', combined_image)

    # Mostrar streams de infrarrojo
    if infrared_image_left is not None and infrared_image_right is not None:
        combined_infrared = np.hstack((infrared_image_left, infrared_image_right))
        cv2.imshow('Infrared Streams (Left & Right)', combined_infrared)
    else:
        if infrared_image_left is not None:
            cv2.imshow('Infrared Stream Left', infrared_image_left)
        if infrared_image_right is not None:
            cv2.imshow('Infrared Stream Right', infrared_image_right)

def depth_to_pointcloud(depth_image, intrinsics):
    """
    Convierte una imagen de profundidad a una nube de puntos 3D.
    depth_image: np.ndarray (H, W), profundidad en milímetros o metros
    intrinsics: dict con fx, fy, cx, cy
    return: np.ndarray (N, 3) nube de puntos (x, y, z)
    """
    h, w = depth_image.shape
    i, j = np.indices((h, w))
    z = depth_image.astype(np.float32)
    # Si está en milímetros, pásalo a metros
    if z.max() > 100:  # heurística simple
        z = z / 1000.0
    x = (j - intrinsics['cx']) * z / intrinsics['fx']
    y = (i - intrinsics['cy']) * z / intrinsics['fy']
    points = np.stack((x, y, z), axis=-1)
    # Filtra puntos inválidos (z==0)
    mask = z > 0
    return points[mask]

def create_pointcloud_message(points, sensor_msgs, frame_id="camera", stamp=None):
    """
    Convierte un array Nx3 (float32) a un mensaje PointCloud Cap'n Proto.
    - points: np.ndarray (N, 3)
    - sensor_msgs: módulo capnp cargado (sensor_msgs.capnp)
    - frame_id: string
    - stamp: timestamp en nanosegundos (opcional)
    """
    N = points.shape[0]
    msg = sensor_msgs.PointCloud.new_message()
    msg.height = 1
    msg.width = N

    # Header
    header = msg.header
    header.frameId = frame_id
    header.stampSec = stamp if stamp is not None else int(time.time() * 1e9)

    # Campos (x, y, z)
    fields = msg.init("fields", 3)
    field_names = ["x", "y", "z"]
    for i, name in enumerate(field_names):
        fields[i].name = name
        fields[i].offset = i * 4
        fields[i].datatype = 7  # FLOAT32 (igual que sensor_msgs/PointField.FLOAT32)
        fields[i].count = 1

    msg.isBigendian = False
    msg.pointStep = 12  # 3 * 4 bytes (x, y, z)
    msg.rowStep = N * 12
    msg.isDense = True

    # Serializar los datos (x, y, z) como float32
    data = points.astype(np.float32).tobytes()
    msg.data = data

    return msg

def create_image_data_message(data, encoding):
    """
    Crear un mensaje ImageData a partir de una imagen.
    """
    image = data['image']
    
    _, encoded_image = cv2.imencode(f'.{encoding}', image)
    
    message = d435i.ImageData.new_message()
    message.width = image.shape[1]
    message.height = image.shape[0]
    message.encoding = encoding
    message.data = image.tobytes()
    message.timestamp = int(data['timestamp'] * 1_000_000) 
    return message

def create_imu_data_message(accel, gyro):
    """
    Crear un mensaje ImuData a partir de datos del acelerómetro y giroscopio.
    """
    message = d435i.ImuData.new_message()

    if accel:
        # Inicializar el grupo 'accel' y asignar valores
        accel_group = message.init('accel')
        accel_group.x, accel_group.y, accel_group.z = accel['data']
        accel_group.timestamp = int(accel['timestamp'] * 1_000_000)  # Convertir a nanosegundos

    if gyro:
        # Inicializar el grupo 'gyro' y asignar valores
        gyro_group = message.init('gyro')
        gyro_group.x, gyro_group.y, gyro_group.z = gyro['data']
        gyro_group.timestamp = int(gyro['timestamp'] * 1_000_000)  # Convertir a nanosegundos

    return message

def create_infrared_data_message(data):
    """
    Crear un mensaje InfraredData a partir de una imagen infrarroja.
    """
    image = data['image']
    
    message = d435i.InfraredData.new_message()
    message.width = image.shape[1]
    message.height = image.shape[0]
    message.data = image.tobytes()
    message.timestamp = int(data['timestamp'] * 1_000_000) 
    return message

def create_camera_message():
    """
    Crear un mensaje CameraData a partir de datos de la cámara.
    """
    message = d435i.CameraData.new_message()
    
    # Inicializar los grupos de datos
    message.color = d435i.ColorData.new_message()
    message.depth = d435i.DepthData.new_message()
    message.infrared_left = d435i.InfraredData.new_message()
    message.infrared_right = d435i.InfraredData.new_message()

    return message

if __name__ == "__main__":
    # Cargar el archivo Cap'n Proto una vez
    d435i = load_msg('d435i')
    sensor_msgs = load_msg('sensor_msgs')

    # Crear un PublisherNode para cada tipo de mensaje
    color_publisher = PublisherNode('ColorPublisher', d435i.ImageData)
    depth_publisher = PublisherNode('DepthPublisher', d435i.ImageData)
# infrared_publisher = PublisherNode('InfraredPublisher', d435i.InfraredData)
    imu_publisher = PublisherNode('IMUPublisher', d435i.ImuData)
    pointcloud_publisher = PublisherNode('PointCloudPublisher', sensor_msgs.PointCloud)

    # Inicializar el RealSenseHandler
    realsense = RealSenseHandler()
    # print(realsense.available_streams())
    realsense.enable_streams(
        depth={'enabled': True, 'width': 640, 'height': 480, 'fps': 30},
        color={'enabled': True, 'width': 424, 'height': 240, 'fps': 60},
        infrared_left={'enabled': False, 'width': 640, 'height': 480, 'fps': 30},
        infrared_right={'enabled': False, 'width': 640, 'height': 480, 'fps': 30},
        imu_accel={'enabled': False},
        imu_gyro={'enabled': False},
    )
    realsense.start()

    # Obtener los intrínsecos del stream de profundidad
    depth_intrinsics = realsense.get_depth_intrinsics()

    # Lista para almacenar los tiempos desde captura hasta publicación
    capture_to_publish_times = []

    try:
        while True:
            # Obtener el tiempo inicial antes de capturar la imagen
            start_time = time.time()

            # Obtener frames de la cámara
            images = realsense.get_frames()

            # Procesar y publicar datos de color
            if images['color']['image'] is not None:
                # Medir el tiempo después de obtener la imagen
                capture_time = time.time()

                # Crear el mensaje
                color_message = create_image_data_message(images['color'], "jpg")

                # Publicar el mensaje
                color_publisher.publish(color_message)

                # Medir el tiempo después de publicar
                publish_time = time.time()

                # Calcular los tiempos
                capture_to_publish_time = publish_time - capture_time
                total_time = publish_time - start_time

                # Almacenar el tiempo en la lista
                capture_to_publish_times.append(capture_to_publish_time)

                # Mostrar los tiempos en la consola
                # print(f"Tiempo desde captura hasta publicación: {capture_to_publish_time:.4f} segundos")
                # print(f"Tiempo total desde inicio hasta publicación: {total_time:.4f} segundos")

                # Calcular y mostrar el promedio cada 10 iteraciones
                if len(capture_to_publish_times) % 10 == 0:
                    average_time = sum(capture_to_publish_times) / len(capture_to_publish_times)
                    print(f"Promedio de tiempo desde captura hasta publicación: {average_time:.4f} segundos")

            # Procesar y publicar datos de profundidad
            if images['depth']['image'] is not None:
                depth_message = create_image_data_message(images['depth'], "png")
                depth_publisher.publish(depth_message)

                # Nube de puntos
                depth_image = images['depth']['image']
                pointcloud = depth_to_pointcloud(depth_image, depth_intrinsics)
                pc_msg = create_pointcloud_message(pointcloud, sensor_msgs, frame_id="camera")
                pointcloud_publisher.publish(pc_msg)

            # Procesar y publicar datos de IMU
            accel, gyro = realsense.get_motion_data()
            if accel is not None and gyro is not None:
                imu_message = create_imu_data_message(accel, gyro)
                imu_publisher.publish(imu_message)

            # Esperar un breve momento para evitar saturar el CPU
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Publicadores detenidos manualmente.")

        # Calcular y mostrar el promedio final
        if capture_to_publish_times:
            average_time = sum(capture_to_publish_times) / len(capture_to_publish_times)
            print(f"Promedio final de tiempo desde captura hasta publicación: {average_time:.4f} segundos")

    finally:
        realsense.stop()
        cv2.destroyAllWindows()

