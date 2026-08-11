extends Node3D

@export var body: RigidBody3D

var g = Vector3(0.0, -9.81, 0.0)
var pos_x_pre = 0.0
var pos_y_pre = 0.0
var pos_z_pre = 0.0
var vel_x_pre = 0.0
var vel_y_pre = 0.0
var vel_z_pre = 0.0

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

func _physics_process(delta: float) -> void:
	var pos_x = body.position.x
	var pos_y = body.position.y
	var pos_z = body.position.z
	var vel_x = (pos_x - pos_x_pre)/delta
	var vel_y = (pos_y - pos_y_pre)/delta
	var vel_z = (pos_z - pos_z_pre)/delta
	
	var ax = (vel_x - vel_x_pre)/delta
	var ay = (vel_y - vel_y_pre)/delta
	var az = (vel_z - vel_z_pre)/delta
	
	var a = global_transform.basis.inverse()*Vector3(ax, ay, az)
	
	var f = a - g
	#print("fx ",f.x)
	#print("fy ",f.y)
	#print("fz ",f.z)
	#print("posx ",pos_x)
	#print("posy ",pos_y)
	#print("posz ",pos_z)
	DDS.publish("pos_x", DDS.DDS_TYPE_FLOAT, pos_x)
	DDS.publish("pos_y", DDS.DDS_TYPE_FLOAT, pos_y)
	DDS.publish("pos_z", DDS.DDS_TYPE_FLOAT, pos_z)
	DDS.publish("vel_x", DDS.DDS_TYPE_FLOAT, vel_x)
	DDS.publish("vel_y", DDS.DDS_TYPE_FLOAT, vel_y)
	DDS.publish("vel_z", DDS.DDS_TYPE_FLOAT, vel_z)
	DDS.publish("a_x", DDS.DDS_TYPE_FLOAT, f.x)
	DDS.publish("a_y", DDS.DDS_TYPE_FLOAT, f.y)
	DDS.publish("a_z", DDS.DDS_TYPE_FLOAT, f.z)

	pos_x_pre = pos_x
	pos_y_pre = pos_y
	pos_z_pre = pos_z
	vel_x_pre = vel_x
	vel_y_pre = vel_y
	vel_z_pre = vel_z


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:	
	pass
