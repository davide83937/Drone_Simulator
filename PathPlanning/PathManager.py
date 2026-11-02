import os
import math
import json
import socket
import struct
import time
from abc import ABC, abstractmethod

# --- CONFIGURAZIONE AMBIENTE ROS 2 WINDOWS ---
extra_dll_dirs = [
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\Library\bin",
]
for directory in extra_dll_dirs:
    if os.path.isdir(directory):
        os.add_dll_directory(directory)
        os.environ["PATH"] = directory + os.pathsep + os.environ.get("PATH", "")

prefix_path = (
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces;"
    r"C:\Users\david\OneDrive\Desktop\ros2-windows"
)
os.environ["AMENT_PREFIX_PATH"] = prefix_path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String, Bool  # Aggiunto Bool per il comando di aggancio
from geometry_msgs.msg import Vector3

from custom_interfaces.action import MoveAxis
from Obstacle import PathPlanner, ObstacleAABB
from PathPlanning.getObstacles import get_obstacles_path_manager
from PathPlanning.obstacle_config import ObstacleConfig

# --- COSTANTI DI MISSIONE ---
SMOKE_COVERAGE_THRESHOLD = 50.0  # Copertura desiderata


# =============================================================================
# DEFINIZIONE DEGLI STATI (STATE PATTERN)
# =============================================================================

class DroneState(ABC):
    def __init__(self, context):
        self.context = context

    @abstractmethod
    def on_enter(self):
        pass

    @abstractmethod
    def on_update(self):
        pass

    @abstractmethod
    def on_exit(self):
        pass

    def on_obstacle_detected(self, msg_str, wp_intersecato):
        pass


# --- STATO 1: Normale volo (raggiungimento target tramite Dijkstra) ---
class NormalFlightState(DroneState):
    def on_enter(self):
        self.context.get_logger().info("[FSM] -> STATO 1: Volo Waypoint Attivo.")
        self.context.is_busy = False
        self.context.current_goal_handle = None
        self.context.send_next_waypoint()

    def on_update(self):
        if self.context.current_wp_index == 0:
            return

        bottom = self.context.last_bottom
        if bottom.get("cross_detected", False):
            self.context.get_logger().info("[FSM] Croce rossa confermata al suolo! Transizione a BoxDockingStateHigh.")
            self.context.change_state(BoxDockingStateHigh(self.context))
            return

        smoke = self.context.last_smoke
        if smoke.get("detected", False) and smoke.get("coverage", 0.0) < SMOKE_COVERAGE_THRESHOLD:
            self.context.get_logger().info("[FSM] Segnale fumo blu avvistato! Transizione a SmokeTrackingState.")
            self.context.change_state(SmokeTrackingState(self.context))
            return

    def on_obstacle_detected(self, msg_str, wp_intersecato):
        self.context.get_logger().warning("[FSM] Ostacolo intralcia la rotta! Transizione a DijkstraReplanState.")
        self.context.change_state(DijkstraReplanState(self.context, msg_str, wp_intersecato))

    def on_exit(self):
        pass


# --- STATO 2: Calcolo o ricalcolo Dijkstra ---
class DijkstraReplanState(DroneState):
    def __init__(self, context, msg_str=None, wp_intersecato=None):
        super().__init__(context)
        self.msg_str = msg_str
        self.wp_intersecato = wp_intersecato

    def on_enter(self):
        self.context.get_logger().info("[FSM] -> STATO 2: Arresto ed esecuzione ricalcolo Dijkstra.")
        if self.msg_str:
            self.context.blocking_obs_pub.publish(self.msg_str)
        self.context.is_replanning = True

        self.context.cancel_motion(on_stopped_callback=self._on_stop_complete)

    def _on_stop_complete(self):
        if self.msg_str is not None:
            self.context.get_logger().info("[FSM] Ostacolo rilevato: avvio retromarcia di sicurezza...")
            self.context.execute_backup(done_callback=self._execute_replan)
        else:
            self.context.get_logger().info("[FSM] Nessun ostacolo bloccante: ricalcolo dalla posizione attuale...")
            self._execute_replan()

    def _execute_replan(self):
        self.context.get_logger().info(
            f"[FSM] Ricalcolo Dijkstra da {self.context.drone_pos} a {self.context.active_goal}...")
        new_path = self.context.planner.dijkstra(
            self.context.drone_pos, self.context.active_goal, initial_yaw=self.context.drone_yaw
        )
        if not new_path:
            self.context.get_logger().error("[FSM] CRITICO: Nessun percorso alternativo trovato!")
            rclpy.shutdown()
            return

        self.context.path = new_path
        self.context.current_wp_index = 0
        self.context.is_replanning = False
        self.context.get_logger().info(f"[FSM] Nuovo percorso trovato ({len(new_path)} WP).")

        # Se stiamo tornando, riprendiamo il volo verso casa, altrimenti volo normale
        if self.context.is_returning:
            self.context.change_state(ReturnFlightState(self.context))
        else:
            self.context.change_state(NormalFlightState(self.context))

    def on_update(self):
        pass

    def on_exit(self):
        pass


# --- STATO 3: Seguire segnale fumo blu ---
class SmokeTrackingState(DroneState):
    def __init__(self, context):
        super().__init__(context)
        self.last_step_time = 0.0
        self.last_smoke_seen_time = time.time()

    def on_enter(self):
        self.context.get_logger().info("[FSM] -> STATO 3: Inseguimento Fumo Blu.")
        self.context.cancel_motion()
        self.last_smoke_seen_time = time.time()

    def on_update(self):
        if self.context.last_bottom.get("cross_detected", False):
            self.context.get_logger().info("[FSM] Croce individuata sotto! Transizione a BoxDockingStateHigh.")
            self.context.change_state(BoxDockingStateHigh(self.context))
            return

        smoke = self.context.last_smoke
        now = time.time()

        if smoke.get("detected", False):
            self.last_smoke_seen_time = now
        else:
            if now - self.last_smoke_seen_time > 2.5:
                self.context.get_logger().warning("[FSM] Segnale fumo perso da oltre 2.5s. Ricalcolo rotta.")
                self.context.change_state(DijkstraReplanState(self.context))
                return
            return

        coverage = smoke.get("coverage", 0.0)
        if coverage >= SMOKE_COVERAGE_THRESHOLD:
            self.context.get_logger().info(f"[FSM] Copertura fumo {coverage:.1f}% >= {SMOKE_COVERAGE_THRESHOLD}%.")
            if smoke.get("box_nearby", False) or self.context.last_bottom.get("box_detected", False):
                self.context.change_state(BoxDockingStateHigh(self.context))
            return

        if not self.context.is_busy and (now - self.last_step_time >= 0.5):
            self.last_step_time = now
            err_x = smoke.get("err_x", 0.0)
            target_yaw = self.context.drone_yaw - (err_x * math.radians(20.0))

            step = 3.0
            new_x = self.context.drone_pos[0] + math.sin(target_yaw) * step
            new_z = self.context.drone_pos[2] - math.cos(target_yaw) * step
            new_y = self.context.drone_pos[1]
            self.context.send_direct_pose(new_x, new_y, new_z, target_yaw)

    def on_exit(self):
        pass


# --- STATO 4: Macro-Allineamento a 15m ---
class BoxDockingStateHigh(DroneState):
    def __init__(self, context):
        super().__init__(context)
        self.align_start_time = None
        self.last_seen_time = time.time()
        self.last_goal_time = 0.0
        self.is_braked = False
        self.focal_px = 320.0 / math.tan(math.radians(30.0))
        self.tolerance_px = 120.0

    def on_enter(self):
        self.context.get_logger().info("[FSM] -> STATO 4: Bersaglio avvistato! Frenata attiva a 15m...")
        self.align_start_time = None
        self.last_seen_time = time.time()
        self.last_goal_time = time.time()
        self.is_braked = False

        curr = self.context.drone_pos
        self.context.is_busy = False
        self.context.send_direct_pose(curr[0], 15.0, curr[2], self.context.drone_yaw)

    def on_update(self):
        now = time.time()
        bottom = self.context.last_bottom
        has_cross = bottom.get("cross_detected", False)

        if not self.is_braked:
            if now - self.last_goal_time >= 1.2:
                self.is_braked = True
                self.context.get_logger().info("[FSM] Drone stabilizzato a 15m. Avvio macro-centratura.")
            return

        if not has_cross:
            self.align_start_time = None
            if now - self.last_seen_time > 3.0:
                self.context.get_logger().warning("[FSM] Bersaglio perso! Ritorno a SmokeTracking.")
                self.context.change_state(SmokeTrackingState(self.context))
            return
        self.last_seen_time = now

        err_x = bottom.get("err_x_px", 0.0)
        err_y = bottom.get("err_y_px", 0.0)
        h_drone = max(2.0, self.context.drone_pos[1])

        delta_x_loc = -(err_x * h_drone) / self.focal_px
        delta_z_loc = (err_y * h_drone) / self.focal_px

        delta_x_loc = max(-1.5, min(1.5, delta_x_loc))
        delta_z_loc = max(-1.5, min(1.5, delta_z_loc))

        yaw = self.context.drone_yaw
        global_dx = (delta_x_loc * math.cos(yaw)) - (delta_z_loc * math.sin(yaw))
        global_dz = (delta_x_loc * math.sin(yaw)) + (delta_z_loc * math.cos(yaw))

        target_x = self.context.drone_pos[0] + global_dx
        target_z = self.context.drone_pos[2] + global_dz

        err_dist = bottom.get("total_err_px", 999.0)
        is_centered = err_dist <= self.tolerance_px

        if is_centered:
            if self.align_start_time is None:
                self.align_start_time = now
                self.context.get_logger().info(
                    f"[FSM] Macro-allineamento OK (Err: {int(err_dist)}px). Raffinamento in corso...")
                self.last_goal_time = now
                self.context.is_busy = False
                self.context.send_direct_pose(target_x, 15.0, target_z, yaw)
            else:
                elapsed = now - self.align_start_time

                if now - self.last_goal_time >= 1.5:
                    self.last_goal_time = now
                    self.context.is_busy = False
                    self.context.send_direct_pose(target_x, 15.0, target_z, yaw)

                if elapsed >= 5.0:
                    self.context.get_logger().info("[FSM] Bersaglio confermato a 15m. Inizio discesa verso 6m.")
                    self.context.change_state(DroneDescentState(self.context))
                    return
        else:
            self.align_start_time = None
            if not self.context.is_busy and (now - self.last_goal_time >= 2.0):
                self.last_goal_time = now
                self.context.is_busy = False
                self.context.send_direct_pose(target_x, 15.0, target_z, yaw)

    def on_exit(self):
        pass


# --- STATO INTERMEDIO: Discesa a 6 metri ---
class DroneDescentState(DroneState):
    def __init__(self, context):
        super().__init__(context)
        self.step_altitudes = [10.0, 6.0]
        self.current_step = 0
        self.last_step_time = 0.0

    def on_enter(self):
        self.context.get_logger().info("[FSM] -> DISCESA: Scaletta quote [10.0m -> 6.0m]")
        self.current_step = 0
        self._command_next_altitude()

    def _command_next_altitude(self):
        target_alt = self.step_altitudes[self.current_step]
        self.last_step_time = time.time()
        curr = self.context.drone_pos
        self.context.get_logger().info(f"[FSM] Discesa verso quota: {target_alt:.1f}m...")
        self.context.is_busy = False
        self.context.send_direct_pose(curr[0], target_alt, curr[2], self.context.drone_yaw)

    def on_update(self):
        now = time.time()
        current_alt = self.context.drone_pos[1]
        target_alt = self.step_altitudes[self.current_step]

        if abs(current_alt - target_alt) <= 1.5 or (now - self.last_step_time >= 3.0):
            if self.current_step < len(self.step_altitudes) - 1:
                self.current_step += 1
                self._command_next_altitude()
            else:
                self.context.get_logger().info("[FSM] Quota 6m raggiunta! Transizione a BoxDockingStateLow.")
                self.context.change_state(BoxDockingStateLow(self.context))

    def on_exit(self):
        pass


# --- STATO 4.5: Micro-Allineamento a 6m ---
class BoxDockingStateLow(DroneState):
    def __init__(self, context):
        super().__init__(context)
        self.align_start_time = None
        self.last_seen_time = time.time()
        self.last_goal_time = 0.0
        self.focal_px = 320.0 / math.tan(math.radians(30.0))
        self.tolerance_px = 60.0
        self.last_centered_time = 0.0

    def on_enter(self):
        self.context.get_logger().info(
            f"[FSM] -> STATO 4.5: Micro-Allineamento a 6m (Tolleranza {self.tolerance_px}px)")
        self.align_start_time = None
        self.last_seen_time = time.time()
        self.last_goal_time = time.time()
        self.last_centered_time = time.time()

        curr = self.context.drone_pos
        self.context.is_busy = False
        self.context.send_direct_pose(curr[0], 6.0, curr[2], self.context.drone_yaw)

    def on_update(self):
        bottom = self.context.last_bottom
        has_cross = bottom.get("cross_detected", False)
        now = time.time()
        current_alt = self.context.drone_pos[1]

        if not has_cross:
            if now - self.last_seen_time > 2.5:
                self.context.get_logger().warning("[FSM] Croce persa a bassa quota. Risalita di sicurezza a 15m!")
                self.context.change_state(BoxDockingStateHigh(self.context))
            return
        self.last_seen_time = now

        err_x = bottom.get("err_x_px", 0.0)
        err_y = bottom.get("err_y_px", 0.0)
        err_dist = bottom.get("total_err_px", 999.0)
        h_drone = max(2.0, current_alt)

        delta_x_loc = -(err_x * h_drone) / self.focal_px
        delta_z_loc = (err_y * h_drone) / self.focal_px

        delta_x_loc = max(-1.20, min(1.20, delta_x_loc))
        delta_z_loc = max(-1.20, min(1.20, delta_z_loc))

        yaw = self.context.drone_yaw
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        global_dx = (delta_x_loc * cos_y) - (delta_z_loc * sin_y)
        global_dz = (delta_x_loc * sin_y) + (delta_z_loc * cos_y)

        target_x = self.context.drone_pos[0] + global_dx
        target_z = self.context.drone_pos[2] + global_dz

        is_centered = err_dist <= self.tolerance_px
        is_at_altitude = current_alt <= 7.0

        if is_centered and is_at_altitude:
            self.last_centered_time = now
            if self.align_start_time is None:
                self.align_start_time = now
                self.context.get_logger().info(f"[FSM] Bersaglio centrato a {current_alt:.1f}m! Convergenza fine...")
                self.last_goal_time = now
                self.context.is_busy = False
                self.context.send_direct_pose(target_x, 6.0, target_z, yaw)
            else:
                elapsed = now - self.align_start_time

                if now - self.last_goal_time >= 1.5:
                    self.last_goal_time = now
                    self.context.is_busy = False
                    self.context.send_direct_pose(target_x, 6.0, target_z, yaw)

                if elapsed >= 2.0:
                    self.context.change_state(DockedState(self.context))
                    return
        else:
            if self.align_start_time is not None:
                if (now - self.last_centered_time > 1.0) or not is_at_altitude:
                    reason = "Allineamento perso" if not is_centered else f"Quota eccessiva ({current_alt:.1f}m)"
                    self.context.get_logger().warning(f"[FSM] {reason}. Timer 5s azzerato.")
                    self.align_start_time = None

            if self.align_start_time is None and not self.context.is_busy and (now - self.last_goal_time >= 1.5):
                self.last_goal_time = now
                self.context.is_busy = False
                self.context.send_direct_pose(target_x, 6.0, target_z, yaw)

    def on_exit(self):
        pass


# --- STATO 5: Lancio della Corda e Aggancio ---
class DockedState(DroneState):
    def __init__(self, context):
        super().__init__(context)
        self.start_time = 0.0
        self.hold_x = 0.0
        self.hold_y = 6.0
        self.hold_z = 0.0
        self.hold_yaw = 0.0

    def on_enter(self):
        self.context.get_logger().info("==========================================")
        self.context.get_logger().info("🪝 [FSM] -> STATO 5: AGGANCIO FISICO (TCP)!")
        self.context.get_logger().info("==========================================")

        self.start_time = time.time()
        self.hold_x = self.context.drone_pos[0]
        self.hold_y = 6.0
        self.hold_z = self.context.drone_pos[2]
        self.hold_yaw = self.context.drone_yaw

        # Primo invio TCP
        self.context.send_grab_command()

        self.context.is_busy = False
        self.context.send_direct_pose(self.hold_x, self.hold_y, self.hold_z, self.hold_yaw)

    def on_update(self):
        now = time.time()

        # Invia periodicamente per garantire la ricezione
        self.context.send_grab_command()

        if now - self.start_time > 3.0:
            self.context.get_logger().info("[FSM] Aggancio confermato. Transizione al Trasporto.")
            self.context.change_state(TransportPhaseState(self.context))

    def on_exit(self):
        pass


# --- STATO 6: Fase di Sollevamento a 30m ---
class TransportPhaseState(DroneState):
    def __init__(self, context):
        super().__init__(context)
        self.lift_command_sent = False
        self.target_alt = 30.0

    def on_enter(self):
        self.context.get_logger().info("==========================================")
        self.context.get_logger().info(f"🚁 [FSM] -> STATO 6: SOLLEVAMENTO CARICO (Quota {self.target_alt}m)")
        self.context.get_logger().info("==========================================")
        self.lift_command_sent = False

        # Spegne attivamente la Computer Vision
        self.context.ignore_cv = True
        # Segnala all'architettura che siamo nella fase di rientro
        self.context.is_returning = True

    def on_update(self):
        now = time.time()
        current_alt = self.context.drone_pos[1]

        if not self.lift_command_sent:
            self.lift_command_sent = True
            self.context.is_busy = False
            self.context.send_direct_pose(
                self.context.drone_pos[0],
                self.target_alt,
                self.context.drone_pos[2],
                self.context.drone_yaw
            )
            self.context.get_logger().info(f"[FSM] Trazione in corso... sollevamento a {self.target_alt}m.")

        # Controlla se la quota target è stata raggiunta (tolleranza 1.5m)
        if abs(current_alt - self.target_alt) <= 1.5:
            self.context.get_logger().info("[FSM] Quota 30m raggiunta in sicurezza! Calcolo rotta di ritorno.")
            self.context.change_state(ReturnReplanState(self.context))

    def on_exit(self):
        pass


# --- STATO 7: Calcolo Rotta di Ritorno (Dijkstra) ---
class ReturnReplanState(DroneState):
    def on_enter(self):
        self.context.get_logger().info("[FSM] -> STATO 7: CALCOLO ROTTA DI RITORNO")
        self.context.cancel_motion(on_stopped_callback=self._execute_replan)

    def _execute_replan(self):
        # Il nuovo target diventa la Base di partenza, ma a 30m di quota
        start_x, _, start_z = self.context.start_pos
        self.context.active_goal = (start_x, 30.0, start_z)

        self.context.get_logger().info(
            f"[FSM] Ricalcolo Dijkstra per il ritorno da {self.context.drone_pos} a {self.context.active_goal}...")

        new_path = self.context.planner.dijkstra(
            self.context.drone_pos, self.context.active_goal, initial_yaw=self.context.drone_yaw
        )

        if not new_path:
            self.context.get_logger().warning("[FSM] Dijkstra fallito per il ritorno, vado in linea retta!")
            new_path = [self.context.drone_pos, self.context.active_goal]

        self.context.path = new_path
        self.context.current_wp_index = 0
        self.context.get_logger().info(f"[FSM] Rotta di ritorno pronta ({len(new_path)} WP).")
        self.context.change_state(ReturnFlightState(self.context))

    def on_update(self):
        pass

    def on_exit(self):
        pass


# --- STATO 8: Volo di Ritorno ---
class ReturnFlightState(DroneState):
    def on_enter(self):
        self.context.get_logger().info("[FSM] -> STATO 8: IN VIAGGIO VERSO LA BASE CON IL CARICO...")
        self.context.is_busy = False
        self.context.send_next_waypoint()

    def on_update(self):
        # Verifica se tutti i WP del ritorno sono stati completati
        if self.context.current_wp_index >= len(self.context.path):
            self.context.change_state(MissionCompleteState(self.context))

    def on_obstacle_detected(self, msg_str, wp_intersecato):
        self.context.get_logger().warning("[FSM] Ostacolo sul tragitto di ritorno! Ricalcolo in corso...")
        # Usa DijkstraReplanState, che ora sa come gestire sia andata che ritorno
        self.context.change_state(DijkstraReplanState(self.context, msg_str, wp_intersecato))

    def on_exit(self):
        pass


# --- STATO 9: Missione Completata ---
class MissionCompleteState(DroneState):
    def on_enter(self):
        self.context.get_logger().info("==========================================")
        self.context.get_logger().info("🎉 [FSM] -> STATO 9: MISSIONE COMPLETATA CON SUCCESSO!")
        self.context.get_logger().info("==========================================")

        # Arresta il drone e lo mantiene in hovering stabile
        curr = self.context.drone_pos
        self.context.is_busy = False
        self.context.send_direct_pose(curr[0], curr[1], curr[2], self.context.drone_yaw)

    def on_update(self):
        pass

    def on_exit(self):
        pass


class GodotTCPClient:
    """Client TCP per invio comandi diretti al Drone in Godot."""
    def __init__(self, ip="127.0.0.1", port=5555):
        self.ip = ip
        self.port = port
        self.sock = None

    def _ensure_connected(self):
        if self.sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(0.5)
                self.sock.connect((self.ip, self.port))
            except Exception:
                self.sock = None

    def send_command(self, cmd: str):
        self._ensure_connected()
        if self.sock is not None:
            try:
                self.sock.sendall((cmd + "\n").encode('utf-8'))
                return True
            except Exception:
                self.sock = None
        return False


# =============================================================================
# CONTEXT PRINCIPALE DEL DRONE (AutonomousDroneClient)
# =============================================================================

class AutonomousDroneClient(Node):
    def __init__(self, planner, final_goal, path, start_pos=(0.0, 15.0, 0.0), start_yaw=0.0):
        super().__init__('autonomous_drone_client')
        self._action_client = ActionClient(self, MoveAxis, 'move_robot')

        # Topic LiDAR e Pose
        self.create_subscription(String, 'new_static_obstacle', self.obstacle_callback, 10)
        self.create_subscription(Vector3, 'drone_position', self.live_pos_callback, 10)
        self.create_subscription(Vector3, 'drone_rotation', self.live_rot_callback, 10)
        self.blocking_obs_pub = self.create_publisher(String, 'blocking_obstacle', 10)
        # Connessione TCP al server interno del Drone (porta 5555)
        self.godot_tcp = GodotTCPClient(ip="127.0.0.1", port=5555)
        # Topic Comando di Aggancio
        self.grab_pub = self.create_publisher(Bool, 'drone_grab_cmd', 10)

        # Topic Computer Vision
        self.create_subscription(String, 'cv_smoke_info', self.smoke_callback, 10)
        self.create_subscription(String, 'cv_bottom_info', self.bottom_callback, 10)

        self.planner = planner
        self.final_goal = final_goal
        self.path = path
        self.current_wp_index = 0
        self.drone_pos = start_pos
        self.drone_yaw = start_yaw

        # NUOVE VARIABILI PER IL RITORNO E CV
        self.start_pos = start_pos  # Memorizza la Base per il ritorno
        self.active_goal = final_goal  # Goal Dinamico (andata: smoke, ritorno: base)
        self.is_returning = False  # Flag FSM
        self.ignore_cv = False  # Interruttore Computer Vision

        self.current_goal_handle = None
        self.is_busy = False
        self.is_replanning = False
        self.last_smoke = {}
        self.last_bottom = {}

        # FSM State Machine
        self.current_state = None
        self.change_state(NormalFlightState(self))

        # Timer periodico per update FSM (10 Hz)
        self.create_timer(0.1, self._fsm_tick)

    def change_state(self, new_state: DroneState):
        if self.current_state:
            self.current_state.on_exit()
        self.current_state = new_state
        self.current_state.on_enter()

    def _fsm_tick(self):
        if self.current_state:
            self.current_state.on_update()

    def live_pos_callback(self, msg):
        self.drone_pos = (msg.x, msg.z, msg.y)

    def live_rot_callback(self, msg):
        self.drone_yaw = math.radians(msg.z)

    def smoke_callback(self, msg):
        if self.ignore_cv: return
        try:
            self.last_smoke = json.loads(msg.data)
        except Exception:
            pass

    def bottom_callback(self, msg):
        if self.ignore_cv: return
        try:
            self.last_bottom = json.loads(msg.data)
        except Exception:
            pass

    def send_grab_command(self):
        success = self.godot_tcp.send_command("GRAB")
        if success:
            self.get_logger().info("[TCP] -> Inviato comando 'GRAB' a Godot (porta 5555)")
        else:
            self.get_logger().warning("[TCP] -> Tentativo connessione porta 5555 in corso...")


    def _register_new_obstacle(self, json_str):
        data = json.loads(json_str)
        new_obs = ObstacleAABB(
            x=data['x'], y=data['z'], z=data['y'],
            width=data['w'], depth=data['h'], height=data['d'],
            drone_radius=ObstacleConfig.get_drone_safety_radius(),
        )
        self.planner.obstacles.append(new_obs)
        return new_obs

    def _find_route_collision(self):
        route_points = [self.drone_pos] + [(wp[0], wp[1], wp[2]) for wp in self.path[self.current_wp_index:]]
        for i, p_curr in enumerate(route_points):
            if not self.planner.is_valid(*p_curr):
                return p_curr
            if i < len(route_points) - 1:
                if not self.planner.is_segment_valid(*p_curr, *route_points[i + 1]):
                    return p_curr
        return None

    def obstacle_callback(self, msg):
        if self.is_replanning or not self.path or self.current_wp_index >= len(self.path):
            return

        self._register_new_obstacle(msg.data)
        wp_intersecato = self._find_route_collision()

        if wp_intersecato is not None:
            self.current_state.on_obstacle_detected(msg, wp_intersecato)

    def execute_backup(self, done_callback):
        dist = ObstacleConfig.get_emergency_backup_distance()
        back_x = self.drone_pos[0]
        back_z = self.drone_pos[2]
        back_y = self.drone_pos[1]
        self.send_direct_pose(back_x, back_y, back_z, self.drone_yaw, on_completed=lambda _: done_callback())

    def start_mission(self):
        self.get_logger().info('In attesa del server Action "move_robot"...')
        self._action_client.wait_for_server()
        self.get_logger().info('Server connesso! Inizio missione.')
        if isinstance(self.current_state, NormalFlightState):
            self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.is_replanning or self.is_busy:
            return

        if self.current_wp_index >= len(self.path):
            self.get_logger().info('Tutti i Waypoint di Dijkstra completati!')
            return

        target = self.path[self.current_wp_index]
        self.send_direct_pose(target[0], target[1], target[2], target[3], on_completed=self._on_wp_reached)

    def send_direct_pose(self, x, y, z, yaw, on_completed=None):
        if self.is_busy:
            return

        self.is_busy = True
        goal_msg = MoveAxis.Goal()
        goal_msg.target_position = Vector3(x=float(x), y=float(y), z=float(z))
        goal_msg.target_rotation = float(yaw)

        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._goal_accepted_callback(f, on_completed))

    def _goal_accepted_callback(self, future, on_completed):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.is_busy = False
            self.current_goal_handle = None
            return
        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._internal_result_cb(f, on_completed))

    def _internal_result_cb(self, future, on_completed):
        self.is_busy = False
        self.current_goal_handle = None
        if on_completed:
            on_completed(future)

    def _on_wp_reached(self, future):
        if self.is_replanning:
            return
        result = future.result().result
        status = future.result().status
        if status == 4 and result.success:
            self.drone_pos = (result.final_position.x, result.final_position.z, result.final_position.y)
            self.current_wp_index += 1
            # Prosegue col prossimo WP sia se siamo all'andata, sia se siamo al ritorno
            if isinstance(self.current_state, NormalFlightState) or isinstance(self.current_state, ReturnFlightState):
                self.send_next_waypoint()

    def cancel_motion(self, on_stopped_callback=None):
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            if on_stopped_callback:
                cancel_future.add_done_callback(lambda _: on_stopped_callback())
            self.current_goal_handle = None
        else:
            if on_stopped_callback:
                on_stopped_callback()
        self.is_busy = False


def get_real_start_pos():
    temp_node = rclpy.create_node('temp_pos_reader')
    drone_pos = None

    def pos_callback(msg):
        nonlocal drone_pos
        drone_pos = (msg.x, msg.z, msg.y)

    temp_node.create_subscription(Vector3, 'drone_position', pos_callback, 10)
    print("In attesa della posizione reale dal Server ROS 2...")
    while drone_pos is None and rclpy.ok():
        rclpy.spin_once(temp_node)

    temp_node.destroy_node()
    return drone_pos


def main(args=None):
    rclpy.init(args=args)

    obstacles = get_obstacles_path_manager()
    bounds = {'x': (-800.0, 800.0), 'y': (0.0, 45.0), 'z': (-800.0, 800.0)}
    planner = PathPlanner(bounds=bounds, resolution=15, obstacles=obstacles)

    start_pos = get_real_start_pos()
    print(f"Posizione reale del drone acquisita: {start_pos}")

    quota_volo = 15.0
    pos_in_aria = (start_pos[0], quota_volo, start_pos[2])
    goal_pos = (26.0, quota_volo, -404.0)

    print(f"Calcolo percorso con Dijkstra da {pos_in_aria} a {goal_pos}...")
    path = planner.dijkstra(pos_in_aria, goal_pos, initial_yaw=0.0)

    if not path:
        print("ERRORE: Nessun percorso valido trovato.")
        return

    # Iniezione salita verticale quota 15m
    wp_decollo = (start_pos[0], quota_volo, start_pos[2], 0.0)
    path.insert(0, wp_decollo)

    client = AutonomousDroneClient(planner, goal_pos, path, start_pos=start_pos, start_yaw=0.0)
    client.start_mission()

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()