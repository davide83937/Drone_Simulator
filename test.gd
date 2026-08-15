extends Node
@export var body: RigidBody3D

var n = 0
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


func _physics_process(delta: float) -> void:
	#body.rotation.z += deg_to_rad(0.1)
	#body.position.x += n
	#n +=0.01
	#body.apply_central_impulse(Vector3(0, 10,0))
	pass
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
