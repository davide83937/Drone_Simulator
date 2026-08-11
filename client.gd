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
	

func propN(prop, rot_torque, n):
	if prop:
		prop.apply_torque(prop.global_transform.basis*rot_torque)
		var angular_velocity_vector: Vector3 = prop.angular_velocity
		var omega: float = angular_velocity_vector.y
		var rpm: float = omega * (60.0 / TAU)
		var vertical_force = 0.0001*pow(rpm, 2)
		
		# Converte la forza globale in una spinta locale rispetto all'orientamento dell'elica
		var local_thrust = prop.global_transform.basis * Vector3(0.0, 0.0, vertical_force)
		prop.apply_force(local_thrust)
		
		var t = Vector3(0.0, 0.0, -rot_torque.z)
		body.apply_torque(prop.global_transform.basis*t)

func prop(prop, rot_torque, n):
	if prop:
		prop.apply_torque(prop.global_transform.basis*rot_torque)
		
		var angular_velocity_vector: Vector3 = prop.angular_velocity
		var omega: float = angular_velocity_vector.y
		var rpm: float = omega * (60.0 / TAU)
		var vertical_force = 0.0001*pow(rpm, 2)
		prop.apply_force(Vector3(0.0, vertical_force, 0.0))
		var t = Vector3(0.0, 0.0, -rot_torque.z)
		body.apply_torque(prop.global_transform.basis*t)
		if n == 1:
			pass
			#print(Vector3(0.0, vertical_force, 0.0))
		#print(vertical_force," -> ",n)

func _physics_process(delta: float) -> void:
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)
	var w1 = DDS.read("w1")
	var w2 = DDS.read("w2")
	var w3 = DDS.read("w3")
	var w4 = DDS.read("w4")
	if w1 == null:
		w1 = 0.0
	if w2 == null:
		w2 = 0.0
	if w3 == null:
		w3 = 0.0
	if w4 == null:
		w4 = 0.0
	var v1 = Vector3(0, 0, float(w1))
	var v2 = Vector3(0, 0, float(w2))
	var v3 = Vector3(0, 0, float(-w3))
	var v4 = Vector3(0, 0, float(-w4))

	#print("w1 = ",w1)
	#print("w2 = ",-w2)
	#print("w3 = ",w3)
	#print("w4 = ",-w4)
	var h = body.position.z
	var px = body.position.x
	var py = body.position.y
	#var speedx = body.linear_velocity.x
	#var speedy = body.linear_velocity.y
	#var speedz = body.linear_velocity.z
	
	#print("Altezza: ",h)
	#print("px: ",px)
	#print("py: ",py)
	#print("speedx: ",speedx)
	#print("speedy: ",speedy)
	#print("speedz: ",speedz)
	

	prop(prop1, v1, 1)
	prop(prop2, v2, 2)
	prop(prop3, v3, 3)
	prop(prop4, v4, 4)
