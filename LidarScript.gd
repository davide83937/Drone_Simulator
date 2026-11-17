extends Node3D

@export var enable_noise: bool = false
@export var noise_std_dev: float = 0.05 

@export_category("LiDAR 3D Specs")
@export var max_range: float = 60.0
@export var vertical_channels: int = 16          
@export var vertical_fov: float = 60.0           
@export var horizontal_resolution: int = 512
@export var rotation_hz: float = 10.0            
@export var min_y_threshold: float = 0.2 
@export var lidar_collision_mask: int = 4

@export_category("Debug Visuale")
@export var show_debug_rays: bool = false

# --- SETUP STREAM UDP ---
@export var target_ip: String = "127.0.0.1"
@export var target_port: int = 9999

var _current_azimuth: float = 0.0
var _debug_timer: float = 0.0
var _seen_objects: Dictionary = {} 

# --- VARIABILI DISEGNO LASER ---
var debug_mesh_instance: MeshInstance3D
var debug_mesh: ImmediateMesh
var debug_material: StandardMaterial3D

# --- GESTIONE MULTI-THREADING ---
var _worker_thread: Thread
var _semaphore: Semaphore
var _mutex: Mutex
var _exit_thread: bool = false
var _thread_packet_peer: PacketPeerUDP
var _thread_rng: RandomNumberGenerator

# Struttura dati scambiata: Array di punti globali Vector3 e Transform globale del genitore
var _raw_hits_queue: Array[Vector3] = []
var _parent_global_xform: Transform3D = Transform3D.IDENTITY


func _ready() -> void:
	# Inizializza sincronizzazione e thread
	_mutex = Mutex.new()
	_semaphore = Semaphore.new()
	_worker_thread = Thread.new()
	_exit_thread = false
	
	_worker_thread.start(_thread_worker_loop)
	
	if show_debug_rays:
		debug_mesh_instance = MeshInstance3D.new()
		debug_mesh = ImmediateMesh.new()
		debug_mesh_instance.mesh = debug_mesh
		debug_mesh_instance.top_level = true
		
		debug_material = StandardMaterial3D.new()
		debug_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		debug_material.albedo_color = Color(1.0, 0.0, 0.0, 0.7)
		
		add_child(debug_mesh_instance)


func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
		
	var space_state = get_world_3d().direct_space_state
	var step_angle = (2.0 * PI) * (rotation_hz * delta)
	var columns_this_frame = int((step_angle / (2.0 * PI)) * horizontal_resolution)
	var azimuth_increment = (2.0 * PI) / float(horizontal_resolution)
	
	if show_debug_rays:
		debug_mesh.clear_surfaces()
		debug_mesh.surface_begin(Mesh.PRIMITIVE_LINES, debug_material)
	
	var frame_hits: Array[Vector3] = []
	var start_pos = global_position
	
	for c in range(columns_this_frame):
		var az = _current_azimuth + (c * azimuth_increment)
		
		for i in range(vertical_channels):
			var ev_deg = -vertical_fov / 2.0 + (vertical_fov / max(1.0, vertical_channels - 1.0)) * i
			var ev = deg_to_rad(ev_deg)
			
			var local_dir = Vector3(
				cos(ev) * sin(az), 
				sin(ev), 
				cos(ev) * cos(az)
			).normalized()
			
			var global_dir = global_transform.basis * local_dir
			var end_pos = start_pos + global_dir * max_range
			
			var query = PhysicsRayQueryParameters3D.create(start_pos, end_pos)
			query.collision_mask = lidar_collision_mask
			
			var result = space_state.intersect_ray(query)
			
			if show_debug_rays:
				debug_mesh.surface_add_vertex(start_pos)
				if result:
					debug_mesh.surface_add_vertex(result.position)
				else:
					debug_mesh.surface_add_vertex(end_pos)
			
			if result:
				var collider = result.collider
				if collider != null and ("RigidBodyPavimento" in str(collider.name) or "BOX_MEDICO" in str(collider.name) or str(collider.name).begins_with("Muro")):
					continue
				if result.position.y <= min_y_threshold:
					continue
					
				if collider != null:
					_seen_objects[collider.name] = true
				
				frame_hits.append(result.position)
				
	if show_debug_rays:
		debug_mesh.surface_end()
				
	_current_azimuth = fmod(_current_azimuth + step_angle, 2.0 * PI)
	
	# Passa i punti grezzi al thread tramite mutex e sveglia il semaforo
	if not frame_hits.is_empty():
		_mutex.lock()
		_raw_hits_queue.append_array(frame_hits)
		var p = get_parent()
		if p != null:
			_parent_global_xform = p.global_transform
		_mutex.unlock()
		_semaphore.post()
	
	_debug_timer += delta
	if _debug_timer >= 1.0:
		if not _seen_objects.is_empty():
			print("[LiDAR] Oggetti validi rilevati: ", _seen_objects.keys())
		_seen_objects.clear()
		_debug_timer = 0.0


# --- CICLO DEL THREAD SECONDARIO ---
func _thread_worker_loop() -> void:
	_thread_peer_setup()
	
	while true:
		_semaphore.wait() # Si sospende a costo zero di CPU finché non arrivano punti
		
		_mutex.lock()
		if _exit_thread:
			_mutex.unlock()
			break
			
		var hits_to_process = _raw_hits_queue.duplicate()
		_raw_hits_queue.clear()
		var parent_xform = _parent_global_xform
		_mutex.unlock()
		
		if hits_to_process.is_empty():
			continue
			
		var local_point_cloud = PackedFloat32Array()
		
		for hit_global in hits_to_process:
			# Trasformazione in coordinate locali del genitore (drone)
			var hit_local_pos = parent_xform.affine_inverse() * hit_global
			
			if enable_noise:
				var dist = hit_local_pos.length()
				dist += _thread_rng.randfn(0.0, noise_std_dev)
				hit_local_pos = hit_local_pos.normalized() * dist
				
			local_point_cloud.append(hit_local_pos.x)
			local_point_cloud.append(hit_local_pos.y)
			local_point_cloud.append(hit_local_pos.z)
			
		if not local_point_cloud.is_empty():
			var byte_data = local_point_cloud.to_byte_array()
			_thread_packet_peer.put_packet(byte_data)


func _thread_peer_setup() -> void:
	_thread_rng = RandomNumberGenerator.new()
	_thread_rng.randomize()
	_thread_packet_peer = PacketPeerUDP.new()
	_thread_packet_peer.set_dest_address(target_ip, target_port)


func _exit_tree() -> void:
	# Pulizia sicura del Thread alla chiusura della scena
	if _worker_thread != null and _worker_thread.is_started():
		_mutex.lock()
		_exit_thread = true
		_mutex.unlock()
		_semaphore.post()
		_worker_thread.wait_to_finish()
