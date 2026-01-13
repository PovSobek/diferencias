import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

class MapScanComparator(Node):

    def __init__(self):
        super().__init__('map_scan_comparator')

        # 1. Suscriptores
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # 2. Publicador para visualizar en RViz (Puntos Rojos)
        self.marker_pub = self.create_publisher(Marker, '/dynamic_obstacles', 10)

        # 3. Buffer de Transformaciones (TF2)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_map = None
        self.get_logger().info('Nodo comparador iniciado. Esperando mapa...')

    def map_callback(self, msg):
        """Guardamos el mapa cuando llega (normalmente una vez)."""
        self.current_map = msg
        self.get_logger().info('Mapa recibido y guardado.')

    def scan_callback(self, scan_msg):
        """Procesa el láser y lo compara con el mapa guardado."""
        if self.current_map is None:
            return

        # Buscamos la transformación: Desde el marco del láser -> Marco del mapa
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',                  # Target frame
                scan_msg.header.frame_id, # Source frame (ej: laser_frame)
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'No se pudo transformar TF: {e}')
            return

        dynamic_points = []

        # Iteramos sobre los rangos del láser
        for i, r in enumerate(scan_msg.ranges):
            # Filtramos valores inválidos o infinito
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            # Calculamos el ángulo actual
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Coordenadas polares a cartesianas (en el marco del láser)
            x_local = r * math.cos(angle)
            y_local = r * math.sin(angle)

            # Crear un punto stamped para transformarlo
            p_stamped = PointStamped()
            p_stamped.point.x = x_local
            p_stamped.point.y = y_local
            p_stamped.point.z = 0.0

            # Transformar punto al marco 'map'
            p_map = do_transform_point(p_stamped, transform)
            
            # --- COMPARACIÓN ---
            # Verificamos qué valor tiene el mapa en esa coordenada
            map_val = self.check_map_value(p_map.point.x, p_map.point.y)

            # Lógica: Si el mapa dice "LIBRE" (0) pero el láser chocó, es algo nuevo.
            # Nota: -1 es desconocido, 0 es libre, 100 es ocupado.
            if map_val == 0: 
                dynamic_points.append(p_map.point)

        # Publicar los puntos detectados en RViz
        self.publish_markers(dynamic_points)

    def check_map_value(self, x, y):
        """Devuelve el valor de la celda del mapa en (x,y world coords)."""
        # 1. Calcular delta desde el origen del mapa
        info = self.current_map.info
        dx = x - info.origin.position.x
        dy = y - info.origin.position.y

        # 2. Convertir a índices de rejilla
        col = int(dx / info.resolution)
        row = int(dy / info.resolution)

        # 3. Verificar límites
        if col < 0 or col >= info.width or row < 0 or row >= info.height:
            return -1 # Fuera del mapa

        # 4. Calcular índice lineal array 1D
        index = row * info.width + col
        return self.current_map.data[index]

    def publish_markers(self, points):
        """Publica una lista de puntos como Marker en RViz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1 # Tamaño del punto
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0 # Rojo
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = points
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MapScanComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
