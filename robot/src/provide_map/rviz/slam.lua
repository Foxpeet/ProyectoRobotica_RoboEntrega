include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- Frames específicos para Turtlebot
  map_frame = "map",
  tracking_frame = "base_footprint",  -- Mejor para Turtlebot
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  
  -- Ajustes para mapas grandes
  map_builder.num_background_threads = 6,  -- Usa más CPU para mapas grandes
  pose_graph.optimize_every_n_nodes = 90,  -- Reduce frecuencia de optimización
  pose_graph.global_sampling_ratio = 0.003, -- Menos restricciones globales
  pose_graph.constraint_builder.sampling_ratio = 0.3,
  
  -- Temporización extendida para mapas grandes
  submap_publish_period_sec = 0.5,
  trajectory_publish_period_sec = 0.5,
  lookup_transform_timeout_sec = 1.0,  -- Mayor tiempo de espera
}

-- Configuración específica para LIDAR de Turtlebot (RPLIDAR A1/A2)
TRAJECTORY_BUILDER_2D = {
  min_range = 0.15,  -- Distancia mínima del LIDAR
  max_range = 12.0,  -- Extendido para mapa grande
  missing_data_ray_length = 5.0,
  
  -- Ajustes de resolución para grandes áreas
  submaps = {
    resolution = 0.075,  -- Resolución más gruesa para ahorrar memoria
    num_range_data = 120,  -- Más scans por submapa
  },
  
  -- Filtrado de movimiento más permisivo
  motion_filter = {
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.0),
    max_time_seconds = 10.0
  },
  
  use_imu_data = false,  -- Turtlebot Burger no tiene IMU fiable
  use_online_correlative_scan_matching = true,
  ceres_scan_matcher = {
    occupied_space_weight = 20.0,
    translation_weight = 10.0,
    rotation_weight = 1.0,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 20,
      num_threads = 4,
    }
  }
}

-- Configuración del grafo de poses para grandes mapas
POSE_GRAPH = {
  constraint_builder = {
    min_score = 0.55,  -- Umbral más bajo para grandes distancias
    global_localization_min_score = 0.6,
    loop_closure_rotation_weight = 1e5,
    max_constraint_distance = 15.0,  -- Búsqueda más amplia
  },
  optimization_problem = {
    acceleration_weight = 1.1e3,
    rotation_weight = 1.6e5,
    local_slam_pose_translation_weight = 1.5e5,
    odometry_translation_weight = 1e5,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 6,
    }
  },
  max_num_final_iterations = 200,  -- Más iteraciones para convergencia
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3
}

return options