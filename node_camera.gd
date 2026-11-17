extends Node3D

@export_category("Streaming Settings")
@export var server_ip: String = "127.0.0.1"
@export var server_port: int = 8000
@export var fps_limit: float = 10.0
@export_range(0.1, 1.0) var jpeg_quality: float = 0.5 

@onready var vp_ahead: SubViewport = $SubViewportAhead
@onready var cam_ahead: Camera3D = $SubViewportAhead/CameraAhead
@onready var vp_bottom: SubViewport = $SubViewportBottom
@onready var cam_bottom: Camera3D = $SubViewportBottom/CameraBottom

@onready var http_ahead: HTTPRequest = $HTTPRequestAhead
@onready var http_bottom: HTTPRequest = $HTTPRequestBottom

var _url_ahead: String
var _url_bottom: String

var _send_interval: float
var _elapsed_ahead: float = 0.0
var _elapsed_bottom: float = 0.0

var _ahead_busy: bool = false
var _bottom_busy: bool = false


func _ready() -> void:
	_url_ahead = "http://%s:%d/image/bottom" % [server_ip, server_port]
	_url_bottom = "http://%s:%d/image/ahead" % [server_ip, server_port]
	_send_interval = 1.0 / max(1.0, fps_limit)
	_elapsed_bottom = _send_interval / 2.0

	# Mantieni la tua risoluzione preferita
	vp_ahead.size = Vector2i(640, 320)
	vp_bottom.size = Vector2i(640, 320)

	vp_ahead.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	vp_bottom.render_target_update_mode = SubViewport.UPDATE_ALWAYS

	cam_ahead.rotation_degrees = Vector3.ZERO
	cam_ahead.near = 0.05
	
	cam_bottom.rotation_degrees = Vector3(-90.0, 0.0, 0.0)
	cam_bottom.near = 0.05

	http_ahead.request_completed.connect(_on_ahead_completed)
	http_bottom.request_completed.connect(_on_bottom_completed)


func _physics_process(delta: float) -> void:
	cam_ahead.global_position = global_position
	cam_ahead.global_basis = global_basis

	cam_bottom.global_position = global_position + (global_basis * Vector3(0, -3, 0))
	cam_bottom.global_transform.basis = global_transform.basis.rotated(global_transform.basis.x, -PI / 2.0)

	_elapsed_ahead += delta
	if _elapsed_ahead >= _send_interval:
		_elapsed_ahead = 0.0
		if not _ahead_busy:
			_capture_and_send(vp_ahead, http_ahead, _url_ahead, true)

	_elapsed_bottom += delta
	if _elapsed_bottom >= _send_interval:
		_elapsed_bottom = 0.0
		if not _bottom_busy:
			_capture_and_send(vp_bottom, http_bottom, _url_bottom, false)


func _capture_and_send(vp: SubViewport, client: HTTPRequest, url: String, is_ahead: bool) -> void:
	var img: Image = vp.get_texture().get_image()
	if img.is_empty():
		return

	if is_ahead:
		_ahead_busy = true
	else:
		_bottom_busy = true

	# =====================================================================
	# Lancia l'elaborazione pesante in BACKGROUND su un altro core della CPU
	# =====================================================================
	WorkerThreadPool.add_task(_process_image_in_background.bind(img, client, url))


# Eseguita fuori dal thread della fisica: non blocca più il drone!
func _process_image_in_background(img: Image, client: HTTPRequest, url: String) -> void:
	img.flip_x()
	img.flip_y()
	var jpeg_bytes: PackedByteArray = img.save_jpg_to_buffer(jpeg_quality)
	
	# Richiama il thread principale per spedire la richiesta di rete in modo sicuro
	call_deferred("_send_http", client, url, jpeg_bytes)


func _send_http(client: HTTPRequest, url: String, jpeg_bytes: PackedByteArray) -> void:
	var headers: PackedStringArray = PackedStringArray(["Content-Type: image/jpeg"])
	client.request_raw(url, headers, HTTPClient.METHOD_POST, jpeg_bytes)


func _on_ahead_completed(_res: int, _code: int, _headers: PackedStringArray, _body: PackedByteArray) -> void:
	_ahead_busy = false

func _on_bottom_completed(_res: int, _code: int, _headers: PackedStringArray, _body: PackedByteArray) -> void:
	_bottom_busy = false
