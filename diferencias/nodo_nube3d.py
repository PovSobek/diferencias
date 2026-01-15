import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# Importaciones para TF (Transformaciones)
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# --- CONFIGURACIÓN ---
POINTCLOUD_TOPIC = '/camera1_points'  # Tópico de entrada (Simulación)
MAP_TOPIC = '/mapa_3d_global'         # Tópico de salida (Para ver en RViz)
GLOBAL_FRAME = 'odom'                 # Marco fijo global
OUTPUT_FILENAME = "mi_mapa_3d.pcd"    # Nombre del archivo a guardar

class Mapper3DNode(Node):
    def __init__(self):
        super().__init__('mapeado_3d_node')

        # --- PARÁMETROS ---
        self.process_every_n = 8      # Procesar 1 de cada N mensajes (para no saturar)
        self.voxel_size = 0.20        # 5cm de resolución. Aumentar si va lento (ej. 0.1)
        
        self.msg_count = 0

        # Buffer para escuchar las transformaciones (TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Suscripción a la cámara/sensor
        self.sub = self.create_subscription(
            PointCloud2, POINTCLOUD_TOPIC, self.cloud_cb, 10
        )
        # Publicador para ver el progreso en RViz
        self.pub_map = self.create_publisher(
            PointCloud2, MAP_TOPIC, 10
        )

        # Aquí acumularemos todos los puntos: Matriz vacía Nx3
        self.global_map_points = np.empty((0, 3), dtype=np.float32)

        self.get_logger().info(f"Mapper 3D iniciado. Guardará en '{OUTPUT_FILENAME}' al salir.")

    def voxel_grid_filter(self, points):
        """ Filtro para reducir la cantidad de puntos repetidos en el mismo espacio """
        if points.shape[0] == 0:
            return points

        # Calcular índice de voxel
        voxel_indices = np.floor(points / self.voxel_size).astype(int)
        
        # Quedarse solo con índices únicos
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_indices]

    def cloud_cb(self, msg: PointCloud2):
        # 1. Filtro temporal (ignorar mensajes para ahorrar CPU)
        self.msg_count += 1
        if self.msg_count % self.process_every_n != 0:
            return

        # 2. Buscar transformación (Dónde estaba el robot cuando tomó la foto)
        try:
            # Buscamos la transformación desde el frame del sensor hacia 'odom'
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Si TF falla (común al inicio), ignoramos este mensaje
            return

        # 3. Leer puntos del mensaje ROS
        # 'skip_nans=True' elimina puntos inválidos
        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        new_points_list = [[p[0], p[1], p[2]] for p in gen]
        
        if not new_points_list:
            return

        new_points_np = np.array(new_points_list, dtype=np.float32)

        # 4. Filtrar la nube local (Voxel Grid) antes de transformar
        new_points_np = self.voxel_grid_filter(new_points_np)

        # 5. Aplicar Transformación Manualmente (Rotación + Traslación)
        q = trans.transform.rotation
        t = trans.transform.translation
        
        # Convertir Cuaternión a Matriz de Rotación
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*y*y - 2*z*z,  2*x*y - 2*z*w,      2*x*z + 2*y*w],
            [2*x*y + 2*z*w,      1 - 2*x*x - 2*z*z,  2*y*z - 2*x*w],
            [2*x*z - 2*y*w,      2*y*z + 2*x*w,      1 - 2*x*x - 2*y*y]
        ])
        T_vec = np.array([t.x, t.y, t.z])

        # Aplicar fórmula: P_global = R * P_local + T
        points_global = (np.dot(R, new_points_np.T).T) + T_vec

        # 6. Acumular en el mapa global
        self.global_map_points = np.vstack((self.global_map_points, points_global))
        
        # 7. Filtro periódico al mapa global para que no crezca infinito
        # Solo filtramos si supera los 50.000 puntos para no frenar el PC
        if self.global_map_points.shape[0] > 50000:
             self.global_map_points = self.voxel_grid_filter(self.global_map_points)

        # 8. Publicar resultado parcial (para ver en RViz)
        self.publish_global_map()

    def publish_global_map(self):
        """ Publica el mapa acumulado en ROS """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = GLOBAL_FRAME

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        pc2_msg = pc2.create_cloud(header, fields, self.global_map_points)
        self.pub_map.publish(pc2_msg)

    def save_map_to_file(self, filename):
        """ Función para guardar la nube de puntos en formato .PCD """
        num_points = self.global_map_points.shape[0]
        if num_points == 0:
            self.get_logger().warn("El mapa está vacío. No se guardará nada.")
            return

        self.get_logger().info(f"--- GUARDANDO MAPA ({num_points} puntos) EN {filename} ---")
        
        try:
            with open(filename, 'w') as f:
                # Cabecera estándar PCD (ASCII)
                header = f"""# .PCD v.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""
                f.write(header)
                # Escribir cada punto
                for p in self.global_map_points:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            
            self.get_logger().info(f"✅ ¡ÉXITO! Archivo guardado: {filename}")
        except Exception as e:
            self.get_logger().error(f"❌ Error al guardar el archivo: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Mapper3DNode()
    
    try:
        # El nodo se queda aquí 'pensando' hasta que pulses Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Esto se ejecuta cuando interrumpes el programa
        node.get_logger().info('Interrupción detectada (Ctrl+C).')
        node.save_map_to_file(OUTPUT_FILENAME)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
