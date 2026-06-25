"""Object-level sensor models: camera, LiDAR, radar.

Note: the production implementation is the C++ version exposed as ``sensors_cpp``
(built via pybind11). This pure-Python module is a legacy/reference implementation
retained for the older debug/demo scripts (e.g. debug_avoidance.py).


Each sensor observes agents within its field of view and range and returns noisy
detections. We model object-level returns (not raw point clouds) which is enough
for the fusion / tracking / planning story:

  * LiDAR  — accurate position (x, y), no velocity.
  * Radar  — coarse lateral position, accurate range + radial velocity (Doppler).
  * Camera — accurate bearing, weak range (from apparent size), object class.

All detections are returned in the world frame (the ego pose is known to the
stack), each with a measurement-noise covariance R so the fusion/tracker can
weight them. Sensors can miss a detection (p_miss) and radar can produce clutter.
"""
import numpy as np

rng = np.random.default_rng(0)


def _visible(ego, s, max_range, half_fov):
    dx, dy = s[0] - ego[0], s[1] - ego[1]
    r = np.hypot(dx, dy)
    if r > max_range or r < 1e-3:
        return False, r, 0.0
    bearing = np.arctan2(dy, dx) - ego[2]
    bearing = np.arctan2(np.sin(bearing), np.cos(bearing))
    return abs(bearing) <= half_fov, r, bearing


class Detection:
    __slots__ = ("z", "R", "sensor", "vr", "kind")

    def __init__(self, z, R, sensor, vr=None, kind=None):
        self.z = np.asarray(z, float)   # [x, y] world frame
        self.R = np.asarray(R, float)   # 2x2 covariance
        self.sensor = sensor
        self.vr = vr                    # radial velocity (radar only), m/s
        self.kind = kind                # class (camera only)


class Lidar:
    def __init__(self, max_range=80.0, half_fov=np.radians(60), sigma=0.15, p_miss=0.05):
        self.max_range, self.half_fov, self.sigma, self.p_miss = max_range, half_fov, sigma, p_miss

    def measure(self, ego, objects):
        out = []
        for _, s, _ in objects:
            vis, r, _ = _visible(ego, s, self.max_range, self.half_fov)
            if not vis or rng.random() < self.p_miss:
                continue
            z = s[:2] + rng.normal(0, self.sigma, 2)
            R = np.eye(2) * self.sigma ** 2
            out.append(Detection(z, R, "lidar"))
        return out


class Radar:
    def __init__(self, max_range=120.0, half_fov=np.radians(45),
                 sigma_r=0.4, sigma_lat=1.2, sigma_vr=0.1, p_miss=0.05):
        self.max_range, self.half_fov = max_range, half_fov
        self.sigma_r, self.sigma_lat, self.sigma_vr, self.p_miss = sigma_r, sigma_lat, sigma_vr, p_miss

    def measure(self, ego, objects):
        out = []
        for _, s, _ in objects:
            vis, r, bearing = _visible(ego, s, self.max_range, self.half_fov)
            if not vis or rng.random() < self.p_miss:
                continue
            # accurate range, coarse cross-range -> anisotropic noise along LOS
            ux, uy = np.cos(ego[2] + bearing), np.sin(ego[2] + bearing)
            z = s[:2] + ux * rng.normal(0, self.sigma_r) + np.array([-uy, ux]) * rng.normal(0, self.sigma_lat)
            # build R in world frame: rotate diag(sigma_r^2, sigma_lat^2) by LOS angle
            ang = ego[2] + bearing
            Rd = np.diag([self.sigma_r ** 2, self.sigma_lat ** 2])
            Rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
            R = Rot @ Rd @ Rot.T
            # radial velocity (Doppler): projection of object velocity on LOS
            vr = (s[2] * ux + s[3] * uy) + rng.normal(0, self.sigma_vr)
            out.append(Detection(z, R, "radar", vr=vr))
        return out


class Camera:
    def __init__(self, max_range=70.0, half_fov=np.radians(35),
                 sigma_bearing=np.radians(0.6), range_rel=0.10, p_miss=0.04):
        self.max_range, self.half_fov = max_range, half_fov
        self.sigma_bearing, self.range_rel, self.p_miss = sigma_bearing, range_rel, p_miss

    def measure(self, ego, objects):
        out = []
        for _, s, kind in objects:
            vis, r, bearing = _visible(ego, s, self.max_range, self.half_fov)
            if not vis or rng.random() < self.p_miss:
                continue
            r_meas = r * (1 + rng.normal(0, self.range_rel))     # weak range
            b_meas = bearing + rng.normal(0, self.sigma_bearing)  # accurate bearing
            ang = ego[2] + b_meas
            z = np.array([ego[0] + r_meas * np.cos(ang), ego[1] + r_meas * np.sin(ang)])
            # R: large along range, small across -> rotate
            Rd = np.diag([(self.range_rel * r) ** 2, (r * self.sigma_bearing) ** 2])
            Rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
            R = Rot @ Rd @ Rot.T
            out.append(Detection(z, R, "camera", kind=kind))
        return out


class SensorSuite:
    def __init__(self):
        self.lidar, self.radar, self.camera = Lidar(), Radar(), Camera()

    def measure(self, ego, objects):
        return (self.lidar.measure(ego, objects)
                + self.radar.measure(ego, objects)
                + self.camera.measure(ego, objects))
