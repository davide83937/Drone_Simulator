extends Node3D
var rng = RandomNumberGenerator.new()
@export var body: RigidBody3D

var rot_x_pre = 0.0
var rot_y_pre = 0.0
var rot_z_pre = 0.0


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
		
	# 1. Leggiamo la rotazione globale attuale (in radianti)
	var rot_x = body.rotation.x
	var rot_y = body.rotation.y
	var rot_z = body.rotation.z
	
	# 2. Calcoliamo la velocità angolare globale (derivata rispetto al tempo)
	var gx = (rot_x - rot_x_pre) / delta
	var gy = (rot_y - rot_y_pre) / delta
	var gz = (rot_z - rot_z_pre) / delta
	
	# 3. Trasformiamo la velocità angolare dal riferimento GLOBALE al riferimento LOCALE
	#var g_local = body.transform.basis.inverse() * Vector3(gx, gy, gz)
	
	# 4. Pubblichiamo le velocità angolari locali in gradi al secondo (deg/s)
	#DDS.publish("gyro_x", DDS.DDS_TYPE_FLOAT, rad_to_deg(gx))
	#DDS.publish("gyro_y", DDS.DDS_TYPE_FLOAT, rad_to_deg(gy))
	#DDS.publish("gyro_z", DDS.DDS_TYPE_FLOAT, rad_to_deg(gz))

	# Ottieni la velocità angolare reale (rad/s) nel sistema locale del drone
	var gyro_local = body.global_transform.basis.inverse() * body.angular_velocity
	
	# Pubblica in gradi al secondo per l'EKF
	DDS.publish("gyro_x", DDS.DDS_TYPE_FLOAT, rad_to_deg(gyro_local.x))
	DDS.publish("gyro_y", DDS.DDS_TYPE_FLOAT, rad_to_deg(gyro_local.y))
	DDS.publish("gyro_z", DDS.DDS_TYPE_FLOAT, rad_to_deg(gyro_local.z))

	# 5. Salviamo le rotazioni per il prossimo frame
	rot_x_pre = rot_x
	rot_y_pre = rot_y
	rot_z_pre = rot_z

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:

	
	pass
