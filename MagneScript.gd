extends Node3D
@export var body: RigidBody3D

var B = Vector3(50.0, 0.0, 0.0)*0.000001

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta: float) -> void:
	var B_result = body.transform.basis.inverse()*B
	print("B: ", B_result)
	DDS.publish("b_x", DDS.DDS_TYPE_FLOAT, B_result.x)
	DDS.publish("b_y", DDS.DDS_TYPE_FLOAT, B_result.y)
	DDS.publish("b_z", DDS.DDS_TYPE_FLOAT, B_result.z)

func _process(delta: float) -> void:
	pass
