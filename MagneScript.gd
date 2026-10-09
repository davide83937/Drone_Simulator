extends Node3D

@export var body: RigidBody3D

# --- IMPOSTAZIONI RUMORE (Visibili nell'Inspector) ---
@export var enable_noise: bool = true
@export var noise_std_dev_uT: float = 0.01 # Deviazione standard in microTesla (µT)

var B = Vector3(50.0, 0.0, 0.0) * 0.000001
var rng = RandomNumberGenerator.new()

func _ready() -> void:
	# Inizializza il generatore per avere rumore sempre casuale
	rng.randomize()

func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
		
	# Trasforma il campo magnetico globale nel sistema di riferimento locale del drone
	var B_result = body.global_transform.basis.inverse() * B
	
	# Aggiungi il rumore gaussiano se l'opzione è attiva
	if enable_noise:
		# Convertiamo il rumore da microTesla a Tesla per allinearlo al vettore B
		var noise_factor = noise_std_dev_uT * 0.000001
		
		B_result.x += rng.randfn(0.0, noise_factor)
		B_result.y += rng.randfn(0.0, noise_factor)
		B_result.z += rng.randfn(0.0, noise_factor)

	#print("B: ", B_result)
	
	DDS.publish("b_x", DDS.DDS_TYPE_FLOAT, B_result.x)
	DDS.publish("b_y", DDS.DDS_TYPE_FLOAT, B_result.y)
	DDS.publish("b_z", DDS.DDS_TYPE_FLOAT, B_result.z)

func _process(delta: float) -> void:
	pass
