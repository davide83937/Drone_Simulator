extends Node3D # Oppure il tipo di nodo padre che stai usando

# Collega il RigidBody3D figlio dall'Inspector di Godot
@export var body: RigidBody3D
@export var prop1: RigidBody3D
@export var prop2: RigidBody3D
@export var prop3: RigidBody3D
@export var prop4: RigidBody3D
@export var thrust_coefficient: float = 0.1
var rot_torque: Vector3 = Vector3(0, 0, 25.0)
var rot_torque1: Vector3 = Vector3(0, 0, -25.0)


var i:int = 0
func _ready() -> void:
	DDS.subscribe("w1")
	DDS.subscribe("w2")
	DDS.subscribe("w3")
	DDS.subscribe("w4")
	DDS.subscribe("n")
	


func prop(prop, rot_torque, n):
	if prop:
		# 1. Applica la coppia (ora non sarà mai "negativa" e distruttiva grazie al mixer)
		var torque = prop.global_transform.basis * rot_torque
		#print("torque: ",torque)
		prop.apply_torque(torque)
		#print("torque: ", torque)
		
		
		# 2. Leggi gli RPM fisici
		var angular_velocity_vector: Vector3 = prop.angular_velocity
		var omega: float = angular_velocity_vector.y
		
		# 2. Leggi gli RPM fisici (DEVE ESSERE L'ASSE Z LOCALE, NON LA Y GLOBALE!)
		#var local_angular_vel = prop.global_transform.basis.inverse() * prop.angular_velocity
		#var omega: float = local_angular_vel.z
		
		# --- AGGIUNTA FONDAMENTALE: ATTRITO AERODINAMICO ---
		# Simula la resistenza dell'aria. Regola "0.01" in base alla massa della tua elica.
		# Questo frenerà l'elica quando rot_torque diventa 0.
		var drag_coefficient = 0.05
		#var drag_torque = -sign(omega) * drag_coefficient * abs(omega)
		var v = Vector3(0.0, 0.0, -sign(omega) * prop.angular_velocity.length())
		#print(v)
		var contro_torque = (prop.global_transform.basis*v)*drag_coefficient
		#print("contro_torque: ",contro_torque)
		prop.apply_torque(contro_torque)
		
		var rpm: float = omega * (60.0 / TAU)
	
		# 3. SPINTA FISICA PURA
		# Essendo calcolata con pow(rpm, 2), è indipendente dal segno di rotazione
		# (risolve automaticamente il problema delle eliche che girano in senso orario/antiorario)
		#print(n, ": ", rpm)
		var vertical_force = 0.1 * abs(rpm)
		#print(n, ": ", rpm, " - ",vertical_force)
		var local_thrust = prop.global_transform.basis * Vector3(0.0, 0.0, vertical_force)

		prop.apply_force(local_thrust)
		
		# Definisci un coefficiente per la resistenza dell'aria (da tarare)
		var drag_factor = 0.005 

		# Calcola il vettore di torsione basato sulla velocità dell'elica lungo il suo asse Y locale
		#var local_torque = Vector3(0.0, 0.0, drag_factor * abs(rpm))
		var reaction_torque = body.global_transform.basis.y * (drag_factor * abs(rpm))# * local_torque
		print("r_torque: ", -local_thrust)
		# Applica la torsione al corpo centrale del drone.
		# ATTENZIONE: Le due diagonali devono spingere in versi opposti!
		# Assumendo che w1/w2 siano una diagonale e w3/w4 l'altra:
		reaction_torque = body.global_transform.basis*Vector3(0.0, 0.0, 0.1)

		#if n == 1 or n == 2:
		body.apply_torque(-torque) # Senso antiorario
			#pass
		#elif n == 3 or n == 4:
			#body.apply_torque(-torque)  # Senso orario
			#pass		
		# 4. Contro-coppia al corpo per lo Yaw
		#var t = Vector3(0.0, 0.0, -rot_torque.z)
		#body.apply_torque(prop.global_transform.basis * t)

func _physics_process(delta: float) -> void:
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)
	# --- INIZIO MODIFICA: Svuota la coda leggendo sempre il valore più recente ---
	var w1 = DDS.read("w1")
	var w2 = DDS.read("w2")
	var w3 = DDS.read("w3")	
	var w4 = DDS.read("w4")

	var n = DDS.read("n")

	if n != null:
		#n = n+1
		print(n)

	#print(body.position.z)
	if w1 == null: w1 = 0.0
	if w2 == null: w2 = 0.0
	if w3 == null: w3 = 0.0
	if w4 == null: w4 = 0.0

	var v1 = Vector3(0, 0, float(w1))
	var v2 = Vector3(0, 0, float(w2))
	var v3 = Vector3(0, 0, float(-w3))
	var v4 = Vector3(0, 0, float(-w4))

	var roll = rad_to_deg(body.rotation.y)
	var pitch = rad_to_deg(body.rotation.x)
	var yaw = rad_to_deg(body.rotation.z)
	DDS.publish("roll", DDS.DDS_TYPE_FLOAT, roll)
	DDS.publish("pitch", DDS.DDS_TYPE_FLOAT, pitch)
	DDS.publish("yaw", DDS.DDS_TYPE_FLOAT, yaw)
	
	var h = body.position.z
	print("Altezza: ", h)
	#print("X: ", body.position.x)
	#print("Y: ", body.position.y)
	#print("Z: ", body.position.z)
	print("Roll: ", roll, " Pitch: ",pitch, " Yaw: ",yaw)
	# Passiamo i comandi originali (w1, w2, w3, w4) per verificare l'intenzione del PID
	prop(prop1, v1, 1)
	prop(prop2, v2, 2)
	prop(prop3, v3, 3)
	prop(prop4, v4, 4)
