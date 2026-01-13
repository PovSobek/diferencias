import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import math
import time

class ChangeCandidate:
    def __init__(self, x, y, change_type):
        self.x = x
        self.y = y
        self.type = change_type # 0 = Verde (Nuevo), 1 = Rojo (Desaparecido)
        self.hits = 1           # Cuántas veces lo hemos visto
        self.last_seen = time.time()

class CostmapChangeDetector(Node):
    def __init__(self):
        super().__init__('costmap_change_detector')
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        costmap_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.static_map_sub = self.create_subscription(OccupancyGrid, '/map', self.static_map_callback, map_qos)
        self.local_map_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_map_callback, costmap_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/map_changes_markers', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.static_map = None
        self.marked_locations = []
        self.marker_id_counter = 0

        # --- PARÁMETROS DE AJUSTE ---
        self.tolerance_dist = 1.5       # Metros (tolerancia espacial)
        self.min_hits_to_publish = 5   # Persistencia: Debe detectarse 15 frames seguidos
        self.candidate_group_dist = 0.2 # Agrupar candidatos cercanos (20cm)
        
        # Lista de posibles cambios que estamos evaluando
        self.candidates = [] 

        self.get_logger().info(f"Detector con persistencia iniciado. Requiere {self.min_hits_to_publish} hits.")

    def static_map_callback(self, msg):
        self.static_map = msg

    def local_map_callback(self, local_msg):
        if self.static_map is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', local_msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return

        marker_array = MarkerArray()
        current_time = time.time()

        # Extraer datos para optimizar acceso
        local_w = local_msg.info.width
        local_h = local_msg.info.height
        local_res = local_msg.info.resolution
        local_data = local_msg.data
        local_origin = local_msg.info.origin.position

        global_w = self.static_map.info.width
        global_h = self.static_map.info.height
        global_res = self.static_map.info.resolution
        global_data = self.static_map.data
        global_ox = self.static_map.info.origin.position.x
        global_oy = self.static_map.info.origin.position.y

        step = 4
        OBSTACLE_THRESH = 90
        FREE_THRESH = 10
        
        # Pre-calculo radio en celdas para la tolerancia
        check_radius_cells = int(self.tolerance_dist / global_res)
        local_radius_cells = int(self.tolerance_dist / local_res)

        # 1. BARRIDO DE DETECCIÓN
        for y in range(0, local_h, step):
            for x in range(0, local_w, step):
                idx = y * local_w + x
                local_val = local_data[idx]
                if local_val == -1: continue

                # Coordenadas Globales
                lx = local_origin.x + (x + 0.5) * local_res
                ly = local_origin.y + (y + 0.5) * local_res
                p_local = PointStamped()
                p_local.point.x = lx; p_local.point.y = ly; p_local.point.z = 0.0
                p_global = do_transform_point(p_local, transform)
                gx = p_global.point.x
                gy = p_global.point.y

                # Si ya está marcado definitivamente, ignorar
                if self.is_location_already_marked(gx, gy):
                    continue

                # Coordenadas Grid Global
                grid_x = int((gx - global_ox) / global_res)
                grid_y = int((gy - global_oy) / global_res)

                if not (0 <= grid_x < global_w and 0 <= grid_y < global_h):
                    continue

                global_idx = grid_y * global_w + grid_x
                global_val = global_data[global_idx]

                detected_type = -1 # -1 Nada, 0 Nuevo, 1 Desaparecido

                # Detección Nuevo (Verde)
                if local_val > OBSTACLE_THRESH and global_val < FREE_THRESH:
                    if not self.check_neighborhood(global_data, global_w, global_h, grid_x, grid_y, check_radius_cells, OBSTACLE_THRESH, True):
                        detected_type = 0 # Verde
                
                # Detección Desaparecido (Rojo)
                elif local_val < FREE_THRESH and global_val > OBSTACLE_THRESH:
                    # Aquí usamos neighborhood local para ver si el obstáculo se movió cerca
                    if not self.check_neighborhood(local_data, local_w, local_h, x, y, local_radius_cells, OBSTACLE_THRESH, True):
                        detected_type = 1 # Rojo

                # 2. PROCESAR CANDIDATOS
                if detected_type != -1:
                    self.update_candidates(gx, gy, detected_type, current_time, marker_array)

        # 3. LIMPIEZA DE CANDIDATOS VIEJOS
        # Si un candidato no se ha actualizado en los últimos 1.0 segundos, es ruido -> borrarlo
        self.candidates = [c for c in self.candidates if (current_time - c.last_seen) < 1.0]

        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)

    def update_candidates(self, x, y, c_type, time_now, marker_arr):
        """
        Busca si ya existe un candidato cerca. Si sí, aumenta hits. Si no, crea uno.
        Si hits > threshold, crea el marcador definitivo.
        """
        found = False
        for cand in self.candidates:
            # Distancia al candidato existente
            dist_sq = (cand.x - x)**2 + (cand.y - y)**2
            
            # Si está muy cerca y es del mismo tipo
            if dist_sq < self.candidate_group_dist**2 and cand.type == c_type:
                cand.hits += 1
                cand.last_seen = time_now
                # Actualizamos posición (promedio simple para centrarlo mejor)
                cand.x = (cand.x + x) / 2
                cand.y = (cand.y + y) / 2
                
                # CHEQUEO DE PROMOCIÓN
                if cand.hits == self.min_hits_to_publish:
                    # ¡Confirmado! Es un objeto real
                    r, g, b = (0.0, 1.0, 0.0) if c_type == 0 else (1.0, 0.0, 0.0)
                    self.add_final_marker(marker_arr, cand.x, cand.y, r, g, b)
                
                found = True
                break
        
        if not found:
            # Crear nuevo candidato
            new_cand = ChangeCandidate(x, y, c_type)
            self.candidates.append(new_cand)

    def check_neighborhood(self, data, w, h, cx, cy, radius, threshold, find_obstacles):
        # ... (Misma función de antes) ...
        min_x = max(0, cx - radius)
        max_x = min(w, cx + radius + 1)
        min_y = max(0, cy - radius)
        max_y = min(h, cy + radius + 1)

        for y in range(min_y, max_y):
            for x in range(min_x, max_x):
                idx = y * w + x
                val = data[idx]
                if val == -1: continue 
                if find_obstacles and val > threshold: return True
                if not find_obstacles and val < threshold: return True
        return False

    def is_location_already_marked(self, x, y):
        for (mx, my) in self.marked_locations:
            dist_sq = (x - mx)**2 + (y - my)**2
            if dist_sq < 0.3**2: return True
        return False

    def add_final_marker(self, arr, x, y, r, g, b):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "stable_diffs"
        m.id = self.marker_id_counter
        self.marker_id_counter += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = 0.2
        m.scale.x = 0.15; m.scale.y = 0.15; m.scale.z = 0.4
        m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b
        m.lifetime.sec = 0
        arr.markers.append(m)
        self.marked_locations.append((x, y))
        self.get_logger().info(f"¡Marcador CONFIRMADO en ({x:.2f}, {y:.2f})!")

def main(args=None):
    rclpy.init(args=args)
    node = CostmapChangeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()