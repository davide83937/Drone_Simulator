import numpy as np
import open3d as o3d
import time

class Lidar_Processor:
    def __init__(
        self,
        height_limit: float = -12.0,
        voxel_size: float = 0.1,
        eps: float = 8, #distanza massima entro cui due punti vengono considerati "vicini"
        min_points: int = 10,
        min_radius: float = 2.5,
        max_radius: float = 6.0,
        max_radial_err: float = 1.9,
    ):
        self.height_limit = height_limit
        self.voxel_size = voxel_size
        self.eps = eps
        self.min_points = min_points
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.max_radial_err = max_radial_err
        self._last_debug_t = 0.0
        self.debug_interval = 1.0  # stampa al massimo 1 volta al secondo

    def process(self, lidar_data: np.ndarray):
        now = time.monotonic()
        should_log = (now - self._last_debug_t) >= self.debug_interval
        if should_log:
            self._last_debug_t = now

        if lidar_data is None or len(lidar_data) == 0:
            if should_log:
                print("[DEBUG] lidar_data vuoto")
            return np.empty((0, 3), dtype=np.float32), []

        n_raw = len(lidar_data)
        pts = self.height_filter(lidar_data)
        n_height = len(pts)
        if n_height == 0:
            if should_log:
                print(f"[DEBUG] raw={n_raw} -> 0 dopo height_filter (y < {self.height_limit})")
            return np.empty((0, 3), dtype=np.float32), []

        pts = self.voxel_filter(pts, self.voxel_size)
        n_voxel = len(pts)
        if n_voxel == 0:
            if should_log:
                print(f"[DEBUG] raw={n_raw}, h={n_height} -> 0 dopo voxel_filter")
            return np.empty((0, 3), dtype=np.float32), []

        clusters = self.cluster_points(pts, eps=self.eps, min_points=self.min_points)
        if should_log:
            sizes = [len(c) for c in clusters]
            #print(f"[DEBUG] raw={n_raw} -> h={n_height} -> vox={n_voxel} -> clusters={len(clusters)} {sizes}")

        cylinders = []
        for idx, c_pts in enumerate(clusters):
            cyl = self.is_cylinder(c_pts, debug=should_log, cluster_id=idx)
            if cyl is not None:
                cylinders.append(cyl)

        return pts, cylinders

    def height_filter(self, lidar_data: np.ndarray) -> np.ndarray:
        mask = lidar_data[:, 1] < self.height_limit
        return lidar_data[mask]

    def voxel_filter(self, pts: np.ndarray, voxel_size: float) -> np.ndarray:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        down = pcd.voxel_down_sample(voxel_size=voxel_size)
        return np.asarray(down.points, dtype=np.float32)

    def cluster_points(self, pts: np.ndarray, eps: float = 0.6, min_points: int = 10):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        labels = np.array(
            pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False)
        )
        # labels == -1 sono i punti di rumore (outlier)
        clusters = []
        for lbl in np.unique(labels):
            if lbl == -1:
                continue
            clusters.append(pts[labels == lbl])
        return clusters

    @staticmethod
    def _fit_circle_2d_ls(u: np.ndarray, v: np.ndarray):
        """Stima (cu, cv, radius) di un cerchio 2D ai minimi quadrati."""
        A = np.column_stack([2.0 * u, 2.0 * v, np.ones_like(u)])
        b = u**2 + v**2
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        cu, cv, c0 = sol

        r_sq = c0 + cu**2 + cv**2
        if r_sq <= 1e-8:
            return None

        return float(cu), float(cv), float(np.sqrt(r_sq))

    def is_cylinder(self, cluster_pts: np.ndarray, debug: bool = False, cluster_id: int = -1):
        if cluster_pts is None or len(cluster_pts) < self.min_points:
            if debug:
                """print(
                    f"  [C{cluster_id}] scartato: pochi punti "
                    f"({0 if cluster_pts is None else len(cluster_pts)} < {self.min_points})"
                )"""
            return None

        u = cluster_pts[:, 0].astype(np.float64)
        v = cluster_pts[:, 2].astype(np.float64)

        fit = self._fit_circle_2d_ls(u, v)
        if fit is None:
            if debug:
                print(f"  [C{cluster_id}] scartato: fit cerchio degenere (r_sq <= 0)")
            return None

        cu, cv, radius = fit
        dists = np.hypot(u - cu, v - cv)
        mean_err = float(np.mean(np.abs(dists - radius)))

        if not (self.min_radius <= radius <= self.max_radius):
            if debug:
                """print(
                    f"  [C{cluster_id}] scartato per raggio: R={radius:.3f} "
                    f"fuori [{self.min_radius}, {self.max_radius}], "
                    f"err={mean_err:.3f}, center=({cu:.2f}, {cv:.2f})"
                )"""
            return None

        if mean_err > self.max_radial_err:
            if debug:
                """print(
                    f"  [C{cluster_id}] scartato per errore radiale: err={mean_err:.3f} > {self.max_radial_err}, "
                    f"R={radius:.3f}, center=({cu:.2f}, {cv:.2f})"
                )"""
            return None

        if debug:
            """print(
                f"  [C{cluster_id}] OK cilindro: R={radius:.3f}, "
                f"err={mean_err:.3f}, center=({cu:.2f}, {cv:.2f})"
            )"""

        return {
            "center_xz": (cu, cv),
            "radius": radius,
            "mean_err": mean_err,
        }