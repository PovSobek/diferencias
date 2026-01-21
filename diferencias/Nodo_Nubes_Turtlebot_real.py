import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.duration import Duration

# Importaciones TF
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- CONFIGURACIÓN ---
INPUT_TOPIC = '/depth/points'
OUTPUT_TOPIC = '/mapa_3d_global'
GLOBAL_FRAME = 'odom'
ROBOT_BASE_FRAME = 'base_link'

class MapperDefinitivo(Node):
    def __init__(self):
        super().__init__('mapeador_definitivo')

        # 1. AUTO-TF
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publicar_union_camara()

        # 2. Configuración
        self.decimation = 8       
        self.voxel_size = 0.05    
        self.update_rate = 1.0    

        self.global_map = np.empty((0, 3), dtype=np.float32)
        self.temp_buffer = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            PointCloud2, INPUT_TOPIC, self.callback, qos_profile=qos_reliable
        )
        self.pub = self.create_publisher(PointCloud2, OUTPUT_TOPIC, 10)
        self.timer = self.create_timer(self.update_rate, self.publish_map)

        self.get_logger().info("--- MAPPER DEFINITIVO CON GUARDADO ---")
        self.get_logger().info("Pulsa Ctrl+C para guardar el mapa.")

    def publicar_union_camara(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ROBOT_BASE_FRAME
        t.child_frame_id = 'camera_link' 
        t.transform.translation.x = 0.10
        t.transform.translation.z = 0.25
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)

    def callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME, 
                msg.header.frame_id, 
                rclpy.time.Time()) 
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        raw = list(gen)
        if not raw: return

        try:
            arr = np.array(raw, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
            arr = arr[::self.decimation] 
            points_local = np.column_stack((arr['x'], arr['y'], arr['z']))
        except Exception:
            return

        dist = np.linalg.norm(points_local, axis=1)
        mask = (dist > 0.2) & (dist < 5.5)
        points_local = points_local[mask]
        
        if len(points_local) == 0: return

        q = trans.transform.rotation
        t = trans.transform.translation
        
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*y*y - 2*z*z,  2*x*y - 2*z*w,      2*x*z + 2*y*w],
            [2*x*y + 2*z*w,      1 - 2*x*x - 2*z*z,  2*y*z - 2*x*w],
            [2*x*z - 2*y*w,      2*y*z + 2*x*w,      1 - 2*x*x - 2*y*y]
        ], dtype=np.float32)
        T_vec = np.array([t.x, t.y, t.z], dtype=np.float32)

        points_global = points_local @ R.T + T_vec
        self.temp_buffer.append(points_global)

    def publish_map(self):
        if not self.temp_buffer:
            return

        chunk = np.vstack(self.temp_buffer)
        self.temp_buffer = []

        if self.global_map.shape[0] == 0:
            self.global_map = chunk
        else:
            self.global_map = np.vstack((self.global_map, chunk))

        if len(self.global_map) > 0:
            idx = np.floor(self.global_map / self.voxel_size).astype(int)
            _, unique = np.unique(idx, axis=0, return_index=True)
            self.global_map = self.global_map[unique]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = GLOBAL_FRAME
        
        msg = pc2.create_cloud_xyz32(header, self.global_map)
        self.pub.publish(msg)
        print(f"\rPuntos acumulados: {len(self.global_map)}", end="")

    # --- NUEVA FUNCIÓN DE GUARDADO ---
    def save_map_to_file(self):
        filename = "MAPA_RT_REAL.pcd"
        num_points = self.global_map.shape[0]
        if num_points == 0:
            print("\n Mapa vacío, no se ha guardado nada.")
            return

        print(f"\n\n--- GUARDANDO {num_points} PUNTOS EN '{filename}' ---")
        try:
            with open(filename, 'w') as f:
                # Cabecera PCD estándar
                header = f"""# .PCD v0.7 - Point Cloud Data file format
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
                # Escribir puntos
                for p in self.global_map:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            
            print(f"✅ Archivo guardado correctamente: {filename}")
        except Exception as e:
            print(f"❌ Error guardando fichero: {e}")

def main():
    rclpy.init()
    node = MapperDefinitivo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # --- AQUÍ LLAMAMOS A GUARDAR AL SALIR ---
        node.save_map_to_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
