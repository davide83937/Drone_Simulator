extends Node3D

@export var body: RigidBody3D

# --- IMPOSTAZIONI RUMORE (Visibili nell'Inspector di Godot) ---
@export var enable_noise: bool = true
@export var noise_std_dev: float = 0.01 # Deviazione standard (intensità) in gradi/s

var rng = RandomNumberGenerator.new()

func _ready() -> void:
	# Randomize cambia il seed iniziale per garantire un rumore sempre diverso a ogni avvio
	rng.randomize()

func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
		
	# 1. Ottieni la velocità angolare reale (rad/s) nel sistema locale del drone
	var gyro_local = body.global_transform.basis.inverse() * body.angular_velocity

	# 2. Converti i valori "ideali" in gradi al secondo
	var gx_deg = rad_to_deg(gyro_local.x)
	var gy_deg = rad_to_deg(gyro_local.y)
	var gz_deg = rad_to_deg(gyro_local.z)

	# 3. Aggiungi il rumore gaussiano se l'opzione è attiva
	if enable_noise:
		# randfn(media, deviazione_standard)
		gx_deg += rng.randfn(0.0, noise_std_dev)
		gy_deg += rng.randfn(0.0, noise_std_dev)
		gz_deg += rng.randfn(0.0, noise_std_dev)

	#print("gx: ", gx_deg, " gy: ", gy_deg, " gz: ", gz_deg)	

	# 4. Pubblica i dati (puliti o rumorosi) verso il controller Python
	DDS.publish("gyro_x", DDS.DDS_TYPE_FLOAT, gx_deg)
	DDS.publish("gyro_y", DDS.DDS_TYPE_FLOAT, gy_deg)
	DDS.publish("gyro_z", DDS.DDS_TYPE_FLOAT, gz_deg)

func _process(delta: float) -> void:
	pass
