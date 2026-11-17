extends RigidBody3D

@onready var raycast: RayCast3D = $RayCast3D

const TCP_PORT = 5555
var tcp_server: TCPServer = TCPServer.new()
var active_peers: Array[StreamPeerTCP] = []

var current_rope_joint: PinJoint3D = null
var rope_mesh: MeshInstance3D = null
var attached_box: RigidBody3D = null


func _ready() -> void:
	print("\n==============================================")
	print("[DRONE TCP] Inizializzazione Server TCP...")
	
	raycast.add_exception(self)
	raycast.target_position = Vector3(0, -12.0, 0)
	raycast.enabled = true

	var err = tcp_server.listen(TCP_PORT)
	if err == OK:
		print("[DRONE TCP] ✅ Server TCP attivo e in ascolto sulla porta: ", TCP_PORT)
	else:
		print("[DRONE TCP] ❌ Errore avvio Server TCP (codice: ", err, ")")
	print("==============================================")

var box_attach_local_pos: Vector3 = Vector3.ZERO


func grab_box() -> bool:
	if current_rope_joint != null:
		print("[DRONE] ⚠️ Carico già agganciato.")
		return true

	raycast.force_raycast_update()
	if raycast.is_colliding():
		var hit = raycast.get_collider()

		if hit is RigidBody3D:
			attached_box = hit

			# Salviamo il punto di impatto esatto (sul tetto del box) nello spazio locale del box
			var hit_point = raycast.get_collision_point()
			box_attach_local_pos = attached_box.to_local(hit_point)

			# 1. Creazione del giunto fisico
			current_rope_joint = PinJoint3D.new()
			current_rope_joint.name = "RopeJoint"
			add_child(current_rope_joint)

			# Il perno fisico agisce esattamente sul punto di contatto
			current_rope_joint.global_position = hit_point
			current_rope_joint.node_a = self.get_path()
			current_rope_joint.node_b = attached_box.get_path()

			# 2. Creazione della mesh della fune
			_create_rope_visual()
			print("[DRONE] 🎯 BINGO! Corda agganciata sul punto di contatto del box.")
			return true

	print("[DEBUG RAYCAST] ❌ Raggio a vuoto.")
	return false


func _create_rope_visual() -> void:
	rope_mesh = MeshInstance3D.new()
	var cyl = CylinderMesh.new()
	cyl.top_radius = 0.02
	cyl.bottom_radius = 0.02
	rope_mesh.mesh = cyl
	
	# Aggiungiamo la corda all'albero globale per evitare che erediti le rotazioni del drone
	get_parent().add_child(rope_mesh)
	_update_rope_transform()


func _update_rope_transform() -> void:
	if rope_mesh == null or attached_box == null:
		return

	# Punto di partenza: pancia del drone (origine del RayCast)
	var start_pos = raycast.global_position
	# Punto di arrivo: punto di ancoraggio sul box
	var end_pos = attached_box.to_global(box_attach_local_pos)

	var dir = end_pos - start_pos
	var length = dir.length()
	if length < 0.01:
		return

	(rope_mesh.mesh as CylinderMesh).height = length

	# Posiziona la mesh a metà tra drone e cassa
	var mid_point = (start_pos + end_pos) / 2.0
	
	# Allineamento asse Y del cilindro lungo il vettore 'dir'
	var up_vec = dir.normalized()
	var right_vec = Vector3.RIGHT if abs(up_vec.dot(Vector3.RIGHT)) < 0.9 else Vector3.FORWARD
	var forward_vec = up_vec.cross(right_vec).normalized()
	right_vec = forward_vec.cross(up_vec).normalized()

	rope_mesh.global_transform = Transform3D(Basis(right_vec, up_vec, forward_vec), mid_point)

func _poll_tcp() -> void:
	# Accetta nuove connessioni da Python
	while tcp_server.is_connection_available():
		var peer = tcp_server.take_connection()
		if peer != null:
			active_peers.append(peer)
			print("[DRONE TCP] 🔗 Client Python connesso!")

	# Legge i dati ricevuti
	var i = active_peers.size() - 1
	while i >= 0:
		var peer = active_peers[i]
		peer.poll()
		var status = peer.get_status()

		if status == StreamPeerTCP.STATUS_CONNECTED:
			var bytes_avail = peer.get_available_bytes()
			if bytes_avail > 0:
				var msg = peer.get_utf8_string(bytes_avail).strip_edges()
				print("[DRONE TCP] 📩 Ricevuto comando: ", msg)
				if "GRAB" in msg:
					grab_box()
		elif status == StreamPeerTCP.STATUS_NONE or status == StreamPeerTCP.STATUS_ERROR:
			active_peers.remove_at(i)
		i -= 1

func _physics_process(_delta: float) -> void:
	if Input.is_action_just_pressed("ui_accept"):
		grab_box()

	_poll_tcp()
	_update_rope_transform()
		
