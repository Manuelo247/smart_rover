import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseHandler:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.imu_pipeline = None  # Pipeline separado para la IMU
        self.imu_config = None
        self.pipeline_profile = None

    def available_streams(self):
        """
        Imprimir y devolver los streams disponibles en el pipeline.
        """
        context = rs.context()
        devices = context.query_devices()
        available_streams = {}

        for device in devices:
            print(f"Dispositivo: {device.get_info(rs.camera_info.name)}")
            for sensor in device.query_sensors():
                print(f"  Sensor: {sensor.get_info(rs.camera_info.name)}")
                for profile in sensor.get_stream_profiles():
                    stream = profile.stream_type()
                    format_ = profile.format()
                    fps = profile.fps()

                    # Agrupar por stream y formato
                    key = (stream, format_)
                    if key not in available_streams:
                        available_streams[key] = set()
                    available_streams[key].add(fps)

        # Imprimir los streams agrupados
        for (stream, format_), fps_set in available_streams.items():
            fps_list = sorted(fps_set)
            print(f"    Stream: {stream}, Formato: {format_}, FPS: {fps_list}")

        return available_streams

    def enable_streams(self, **kwargs):
        """
        Habilitar los streams deseados de manera escalable con opciones de resolución y FPS.
        """
        stream_settings = {
            'depth': (rs.stream.depth, rs.format.z16),
            'color': (rs.stream.color, rs.format.yuyv),
            'infrared_left': (rs.stream.infrared, 1, rs.format.y8),
            'infrared_right': (rs.stream.infrared, 2, rs.format.y8),
        }

        imu_enabled = False

        for stream_name, options in kwargs.items():
            if stream_name in stream_settings and options.get('enabled', False):
                stream_type = stream_settings[stream_name][0]
                format_ = stream_settings[stream_name][-1]
                if len(stream_settings[stream_name]) == 3:  # For infrared streams
                    index = stream_settings[stream_name][1]
                    self.config.enable_stream(
                        stream_type, index,
                        options.get('width', 640),
                        options.get('height', 480),
                        format_,
                        options.get('fps', 30)
                    )
                else:
                    self.config.enable_stream(
                        stream_type,
                        options.get('width', 640),
                        options.get('height', 480),
                        format_,
                        options.get('fps', 30)
                    )
            elif stream_name == 'imu_accel' or stream_name == 'imu_gyro':
                if options.get('enabled', False):
                    imu_enabled = True

        # Configurar pipeline separado para la IMU si está habilitada
        if imu_enabled:
            self.imu_pipeline = rs.pipeline()
            self.imu_config = rs.config()
            self.imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
            self.imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    def start(self):
        """
        Iniciar el pipeline.
        """
        self.pipeline_profile = self.pipeline.start(self.config)
        if self.imu_pipeline:
            self.imu_pipeline.start(self.imu_config)

    def stop(self):
        """
        Detener el pipeline.
        """
        self.pipeline.stop()
        if self.imu_pipeline:
            self.imu_pipeline.stop()

    def get_frames(self):
        """
        Obtener los frames
        """
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        infrared_frame_left = frames.get_infrared_frame(1)  # Infrared stream 1
        infrared_frame_right = frames.get_infrared_frame(2)  # Infrared stream 2

        # Obtener imágenes y timestamps
        depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None
        depth_timestamp = depth_frame.get_timestamp() if depth_frame else None

        color_image = np.asanyarray(color_frame.get_data()) if color_frame else None
        color_timestamp = color_frame.get_timestamp() if color_frame else None

        infrared_image_left = np.asanyarray(infrared_frame_left.get_data()) if infrared_frame_left else None
        infrared_left_timestamp = infrared_frame_left.get_timestamp() if infrared_frame_left else None

        infrared_image_right = np.asanyarray(infrared_frame_right.get_data()) if infrared_frame_right else None
        infrared_right_timestamp = infrared_frame_right.get_timestamp() if infrared_frame_right else None

        frames_data = {
            "depth": {"image": depth_image, "timestamp": depth_timestamp},
            "color": {"image": color_image, "timestamp": color_timestamp},
            "infrared_left": {"image": infrared_image_left, "timestamp": infrared_left_timestamp},
            "infrared_right": {"image": infrared_image_right, "timestamp": infrared_right_timestamp},
        }

        return frames_data

    def get_motion_data(self):
        """
        Obtener datos del acelerómetro y giroscopio.
        """
        try:
            if not self.imu_pipeline:
                return None, None

            # Esperar frames del pipeline de la IMU
            frames = self.imu_pipeline.wait_for_frames()

            # Obtener los frames de acelerómetro y giroscopio
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            # Extraer datos de movimiento
            accel_data = accel_frame.as_motion_frame().get_motion_data() if accel_frame else None
            accel_timestamp = accel_frame.get_timestamp() if accel_frame else None

            gyro_data = gyro_frame.as_motion_frame().get_motion_data() if gyro_frame else None
            gyro_timestamp = gyro_frame.get_timestamp() if gyro_frame else None

            # Retornar los datos como tuplas
            accel = {"data": (accel_data.x, accel_data.y, accel_data.z), "timestamp": accel_timestamp} if accel_data else None
            gyro = {"data": (gyro_data.x, gyro_data.y, gyro_data.z), "timestamp": gyro_timestamp} if gyro_data else None

            return accel, gyro
        except Exception as e:
            print(f"Error al obtener datos de movimiento: {e}")
            return None, None

    def get_depth_intrinsics(self):
        """
        Devuelve los intrínsecos de la cámara de profundidad como un diccionario.
        Debes llamar a esta función después de iniciar el pipeline.
        """
        if self.pipeline_profile is None:
            raise RuntimeError("El pipeline no está iniciado. Llama a start() primero.")
        depth_stream = self.pipeline_profile.get_stream(rs.stream.depth)
        intr = depth_stream.as_video_stream_profile().get_intrinsics()
        return {
            'fx': intr.fx,
            'fy': intr.fy,
            'cx': intr.ppx,
            'cy': intr.ppy,
            'width': intr.width,
            'height': intr.height
        }


# Ejemplo de uso
if __name__ == "__main__":
    realsense = RealSenseHandler()
    print("Streams disponibles:")
    available_streams = realsense.available_streams()

    # Habilitar todos los streams
    realsense.enable_streams(
        depth={'enabled': True, 'width': 640, 'height': 480, 'fps': 30},
        color={'enabled': True, 'width': 640, 'height': 480, 'fps': 30},
        infrared_left={'enabled': False, 'width': 640, 'height': 480, 'fps': 30},
        infrared_right={'enabled': False, 'width': 640, 'height': 480, 'fps': 30},
        imu_accel={'enabled': True},
        imu_gyro={'enabled': True},
    )
    realsense.start()

    try:
        while True:
            frames_data = realsense.get_frames()

            depth_image = frames_data["depth"]["image"]
            depth_timestamp = frames_data["depth"]["timestamp"]

            color_image = frames_data["color"]["image"]
            color_timestamp = frames_data["color"]["timestamp"]

            infrared_image_left = frames_data["infrared_left"]["image"]
            infrared_left_timestamp = frames_data["infrared_left"]["timestamp"]

            infrared_image_right = frames_data["infrared_right"]["image"]
            infrared_right_timestamp = frames_data["infrared_right"]["timestamp"]
            
            # Procesar y publicar datos de IMU
            # accel, gyro = realsense.get_motion_data()
            # if accel['data'] is not None and gyro['data'] is not None:
            #     accel = accel['data']
            #     gyro = gyro['data']
            #     print(f"Acelerómetro: x={accel[0]:.2f}, y={accel[1]:.2f}, z={accel[2]:.2f}")
            #     print(f"Giroscopio: x={gyro[0]:.2f}, y={gyro[1]:.2f}, z={gyro[2]:.2f}")


            # Mostrar streams de profundidad y color
            # if depth_image is not None and color_image is not None:
            #     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            #     combined_image = np.hstack((color_image, depth_colormap))
            #     cv2.imshow('RealSense Stream (Color + Depth)', combined_image)

            # Mostrar streams de infrarrojo
            if infrared_image_left is not None and infrared_image_right is not None:
                combined_infrared = np.hstack((infrared_image_left, infrared_image_right))
                cv2.imshow('Infrared Streams (Left & Right)', combined_infrared)
            else:
                if infrared_image_left is not None:
                    cv2.imshow('Infrared Stream Left', infrared_image_left)
                if infrared_image_right is not None:
                    cv2.imshow('Infrared Stream Right', infrared_image_right)

            # Salir si se presiona la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        realsense.stop()
        cv2.destroyAllWindows()
