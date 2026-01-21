import open3d as o3d
import numpy as np
import copy

# --- 1. ALINEACIÓN ---
def alinear_nubes(source, target):
    print("--- 1. Alineando Mapas ---")
    # Aumentamos umbral inicial para atrapar mapas desalineados
    threshold = 2.0 
    trans_init = np.identity(4)
    
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Fase 1: Aproximación
    reg = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    
    # Fase 2: Ajuste Fino (Importante para que lo estático sea gris)
    reg = o3d.pipelines.registration.registration_icp(
        source, target, 0.1, reg.transformation, # 10cm de ajuste fino
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    
    print(f"  -> Alineación Fitness: {reg.fitness:.2f} (Si es < 0.6, la alineación falló)")
    source.transform(reg.transformation)
    return source

# --- 2. LIMPIEZA ---
def limpiar_arquitectura(pcd):
    pcd_obj = copy.deepcopy(pcd)
    
    # Suelo
    plane_model, inliers = pcd_obj.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    # Solo borramos si es un plano horizontal MUY grande (> 2000 puntos)
    if abs(c) > 0.5 and len(inliers) > 2000: 
        pcd_obj = pcd_obj.select_by_index(inliers, invert=True)

    # Paredes
    for _ in range(10): 
        if len(pcd_obj.points) < 500: break
        plane_model, inliers = pcd_obj.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
        # Solo borramos paredes GIGANTES (> 2000 puntos) para no borrar armarios
        if len(inliers) > 2000:
            pcd_obj = pcd_obj.select_by_index(inliers, invert=True)
        else:
            break
            
    pcd_obj, _ = pcd_obj.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd_obj

def get_cluster_props(pcd, indices):
    if len(indices) == 0: return np.array([0,0,0]), 0
    cluster = pcd.select_by_index(indices)
    return cluster.get_center(), len(indices)

# --- 3. ANÁLISIS ---
def analizar_escena(pcd_old, pcd_new):
    print("\n--- 3. Analizando Objetos ---")
    
    # --- PARÁMETROS CRÍTICOS (Ajustados para tu problema) ---
    # Si lo estático sale verde, SUBE esto. (0.25 = 25cm de margen)
    DIST_STATIC = 0.25      
    
    # Si lo movido no sale azul, SUBE esto. (3.5 metros de radio de búsqueda)
    MAX_MOVED_DIST = 3.5    
    
    # Tolerancia de cambio de tamaño (5.0 = el objeto nuevo puede ser 5 veces mas grande/pequeño)
    MAX_VOL_RATIO = 5.0     
    
    MIN_CLUSTER_PTS = 10

    # 1. Separar Estático
    dists_new_to_old = np.asarray(pcd_new.compute_point_cloud_distance(pcd_old))
    
    # Identificar estáticos
    ind_static = np.where(dists_new_to_old < DIST_STATIC)[0]
    pcd_static = pcd_new.select_by_index(ind_static)
    
    # Todo lo que no es estático es candidato a Nuevo
    ind_new_cand = np.where(dists_new_to_old >= DIST_STATIC)[0]
    pcd_new_cand = pcd_new.select_by_index(ind_new_cand)

    # Identificar lo que falta (Viejo no encontrado en Nuevo)
    dists_old_to_new = np.asarray(pcd_old.compute_point_cloud_distance(pcd_new))
    ind_gone_cand = np.where(dists_old_to_new >= DIST_STATIC)[0]
    pcd_gone_cand = pcd_old.select_by_index(ind_gone_cand)

    # 2. Clusterizar (DBSCAN)
    print("  -> Clusterizando candidatos...")
    labels_gone = np.array(pcd_gone_cand.cluster_dbscan(eps=0.35, min_points=MIN_CLUSTER_PTS, print_progress=False))
    labels_new = np.array(pcd_new_cand.cluster_dbscan(eps=0.35, min_points=MIN_CLUSTER_PTS, print_progress=False))

    final_moved = o3d.geometry.PointCloud()
    final_added = o3d.geometry.PointCloud()
    final_removed = o3d.geometry.PointCloud()
    
    lines_points = [] 
    lines_indices = []

    matched_new_clusters = set()
    max_gone = labels_gone.max() if len(labels_gone) > 0 else -1
    max_new = labels_new.max() if len(labels_new) > 0 else -1
    
    print(f"  -> Objetos 'Desaparecidos' potenciales: {max_gone + 1}")
    print(f"  -> Objetos 'Nuevos' potenciales: {max_new + 1}")

    # --- MATCHING LOGIC ---
    for i in range(max_gone + 1):
        idx_g = np.where(labels_gone == i)[0]
        center_g, vol_g = get_cluster_props(pcd_gone_cand, idx_g)
        
        best_idx = -1
        best_score = float('inf') 

        print(f"\n  [?] Buscando match para Objeto Viejo #{i} (Vol: {vol_g} pts)")

        for j in range(max_new + 1):
            if j in matched_new_clusters: continue
            
            idx_n = np.where(labels_new == j)[0]
            center_n, vol_n = get_cluster_props(pcd_new_cand, idx_n)
            
            dist = np.linalg.norm(center_g - center_n)
            ratio = max(vol_g, vol_n) / (min(vol_g, vol_n) + 1e-6)

            # Debug: Ver qué está comparando
            # print(f"      - vs Nuevo #{j}: Dist={dist:.2f}m, Ratio={ratio:.1f}")

            if dist < MAX_MOVED_DIST and ratio < MAX_VOL_RATIO:
                score = dist # Priorizamos cercanía
                if score < best_score:
                    best_score = score
                    best_idx = j
                    best_center_n = center_n

        if best_idx != -1:
            print(f"    >>> MATCH CONFIRMADO: Viejo #{i} se movió a Nuevo #{best_idx} (Dist: {best_score:.2f}m)")
            
            # Pintar Nuevo de AZUL (Movido)
            pts_moved = pcd_new_cand.select_by_index(np.where(labels_new == best_idx)[0])
            pts_moved.paint_uniform_color([0, 0, 1]) 
            final_moved += pts_moved
            
            matched_new_clusters.add(best_idx)
            
            # Línea Negra
            l_idx = len(lines_points)
            lines_points.append(center_g)
            lines_points.append(best_center_n)
            lines_indices.append([l_idx, l_idx + 1])
        else:
            print(f"    XXX No hubo match. Se considera ELIMINADO.")
            pts_removed = pcd_gone_cand.select_by_index(idx_g)
            pts_removed.paint_uniform_color([1, 0, 0]) 
            final_removed += pts_removed

    # Los nuevos restantes son VERDES
    for j in range(max_new + 1):
        if j not in matched_new_clusters:
            pts_added = pcd_new_cand.select_by_index(np.where(labels_new == j)[0])
            pts_added.paint_uniform_color([0, 1, 0]) 
            final_added += pts_added

    # Pintar estáticos de Gris
    pcd_static.paint_uniform_color([0.5, 0.5, 0.5])

    # Líneas
    line_set = o3d.geometry.LineSet()
    if len(lines_points) > 0:
        line_set.points = o3d.utility.Vector3dVector(lines_points)
        line_set.lines = o3d.utility.Vector2iVector(lines_indices)
        line_set.paint_uniform_color([0, 0, 0])

    return pcd_static, final_removed, final_added, final_moved, line_set

def main():
    try:
        old = o3d.io.read_point_cloud("mapa_e.pcd") 
        new = o3d.io.read_point_cloud("mapafinal.pcd")
    except:
        return

    # Muestreo un poco más grueso para ayudar al alineado y reducir ruido
    old = old.voxel_down_sample(voxel_size=0.03)
    new = new.voxel_down_sample(voxel_size=0.03)

    new = alinear_nubes(new, old)
    old_objs = limpiar_arquitectura(old)
    new_objs = limpiar_arquitectura(new)

    static, removed, added, moved, lines = analizar_escena(old_objs, new_objs)

    print("\n--- RESUMEN FINAL ---")
    print(f"ESTÁTICO (Gris):  {len(static.points)}")
    print(f"ELIMINADO (Rojo): {len(removed.points)}")
    print(f"NUEVO (Verde):    {len(added.points)}")
    print(f"MOVIDO (Azul):    {len(moved.points)}")

    o3d.visualization.draw_geometries([static, removed, added, moved, lines],
                                      window_name="Detección de Cambios",
                                      width=1024, height=768)

if __name__ == "__main__":
    main()
