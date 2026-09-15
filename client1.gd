extends Node3D

@export var body: RigidBody3D
# Queste variabili le teniamo per poter magari far girare le mesh visive delle eliche, 
# ma la fisica ora è tutta gestita dal 'body' centrale.
@export var prop1: Node3D 
@export var prop2: Node3D
@export var prop3: Node3D
@export var prop4: Node3D

# Distanza dei motori dal centro (Modifica questo valore per allargare/stringere il quadrato di spinta)
@export var L: float = 1.5 

var n = 0

func _ready() -> void:
	DDS.subscribe("w1")
	DDS.subscribe("w2")
	DDS.subscribe("w3")
	DDS.subscribe("w4")
	DDS.subscribe("n")

# --- LA FUNZIONE DEL PROFESSORE ---
func apply_local_force(force: Vector3, pos: Vector3):
	var pos_global_offset = body.global_transform.basis * pos
	var force_global = body.global_transform.basis * force
	body.apply_force(force_global, pos_global_offset)

func prop(w, n, pos_locale, prop):
	# 1. Spinta (La tua formula originale degli RPM)
	var rpm = w * (60.0 / TAU)
	var vertical_force = 0.1 * abs(rpm)
	#prop.rotate_z(vertical_force/10)
	# Usiamo Z per l'altezza, coerentemente con i tuoi script precedenti
	var force_local = Vector3(0, 0, vertical_force)
	#prop.apply_torque(force_local)
	# Applichiamo la spinta sul vertice del nostro quadrato perfetto
	apply_local_force(force_local, pos_locale)
	
	# 2. Contro-coppia Aerodinamica (YAW)
	var dir = -1.0
	if n == 1 or n == 2:
		dir = 1.0 
	var drag_factor = 0.05 
	
	var torque_local = Vector3(0, 0, vertical_force * drag_factor * dir)
	#var torque_local1 = Vector3(0, 0, 1)
	#print(n, ": ", force_local, ": ", pos_locale)
	body.apply_torque(body.global_transform.basis * torque_local)


func _physics_process(delta: float) -> void:
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)


	var w1 = DDS.read("w1"); if w1 == null: w1 = 0.0
	var w2 = DDS.read("w2"); if w2 == null: w2 = 0.0
	var w3 = DDS.read("w3"); if w3 == null: w3 = 0.0
	var w4 = DDS.read("w4"); if w4 == null: w4 = 0.0
	if w1 != 0.0:
		if n != null:
			n = n+1
			print(n)
		
	# Estrazione angoli originale, semplice e pulita
	#var roll  = rad_to_deg(body.rotation.y)
	#var pitch = rad_to_deg(body.rotation.x)
	var yaw   = rad_to_deg(body.rotation.z) 

	# 1. Otteniamo i vettori fisici direzionali del drone dal mondo globale
	var local_forward = -body.global_transform.basis.y # Dove punta il muso
	var local_left = body.global_transform.basis.x     # Dove punta l'ala sinistra
	
	# 2. Calcoliamo Roll e Pitch usando il seno dell'inclinazione verticale (asse Y globale)
	# Se il muso va su, il pitch è positivo. Se l'ala sinistra va su, il roll è positivo.
	var pitch = rad_to_deg(asin(-local_forward.y))
	var roll  = rad_to_deg(asin(-local_left.y))
	
	#var roll  = rad_to_deg(body.rotation.y)
	#var pitch = rad_to_deg(body.rotation.x)
	# 3. Calcoliamo lo Yaw usando la proiezione del muso sul piano orizzontale (X e Z globali)
	#var yaw = rad_to_deg(atan2(-local_forward.x, -local_forward.z))

	#DDS.publish("roll", DDS.DDS_TYPE_FLOAT, roll)
	#DDS.publish("pitch", DDS.DDS_TYPE_FLOAT, pitch)
	#DDS.publish("yaw", DDS.DDS_TYPE_FLOAT, yaw)
	var x = body.position.x
	var y = body.position.y
	var z = body.position.z
	
	print("x: ", x, " y: ",y, " z: ",z)
	print("Roll: ", roll, " Pitch: ", pitch, " Yaw: ", yaw)

	# --- GEOMETRIA VIRTUALE SIMMETRICA ---
	# Definiamo le 4 posizioni a forma di quadrato perfetto (sul piano X-Y)
	# Assicurati che i segni corrispondano a dove si aspettano i tuoi motori in Python
	var pos1 = Vector3(-L, -L, 0) # Anteriore-Destro (o equivalente)
	var pos2 = Vector3( L, L, 0) # Posteriore-Sinistro
	var pos3 = Vector3( L, -L, 0) # Anteriore-Sinistro
	var pos4 = Vector3(-L, L, 0) # Posteriore-Destro

	# Richiamiamo i motori passando la velocità e la posizione geometrica perfetta
	# Mantieni i segni meno se ti servivano per far girare le eliche nel verso giusto
	prop(float(w1), 1, pos1, prop1)
	prop(float(w2), 2, pos2, prop2)
	prop(float(w3), 3, pos3, prop3)
	prop(float(w4), 4, pos4, prop4)
