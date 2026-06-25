"""Perception / sensor fusion.

Note: the production implementation is the C++ version exposed as ``perception_cpp``
(built via pybind11). This pure-Python module is a legacy/reference implementation
retained for the older debug/demo scripts (e.g. debug_avoidance.py).


Group the per-sensor detections that belong to the same physical object and fuse
them. Position fusion uses inverse-covariance (information-form) weighting, which
is the maximum-likelihood combination of independent Gaussian measurements:

    R_f = ( sum_i R_i^-1 )^-1 ,    z_f = R_f ( sum_i R_i^-1 z_i ).

The object class comes from the camera; a coarse velocity prior comes from the
radar Doppler (radial component only) to warm-start a new track. Output is a list
of fused detections that the world-model tracker consumes each frame.
"""
import numpy as np


class FusedDetection:
    __slots__ = ("z", "R", "kind", "v_prior", "sensors")

    def __init__(self, z, R, kind, v_prior, sensors):
        self.z = z; self.R = R; self.kind = kind
        self.v_prior = v_prior      # [vx, vy] rough init or None
        self.sensors = sensors


def _cluster(dets, gate=9.21):
    """Greedy association of detections by Mahalanobis distance (chi-square gate).

    Euclidean gating fails for the camera, whose along-range error can be many
    metres; using (R_i + R_j) accounts for each sensor's anisotropic uncertainty
    so a high-variance camera detection still associates to the right object.
    """
    clusters, used = [], [False] * len(dets)
    for i in range(len(dets)):
        if used[i]:
            continue
        group = [i]; used[i] = True
        for j in range(i + 1, len(dets)):
            if used[j]:
                continue
            diff = dets[i].z - dets[j].z
            S = dets[i].R + dets[j].R
            d2 = float(diff @ np.linalg.inv(S) @ diff)
            if d2 < gate:
                group.append(j); used[j] = True
        clusters.append(group)
    return clusters


def fuse(dets, gate=9.21):
    out = []
    for group in _cluster(dets, gate):
        info = np.zeros((2, 2)); infz = np.zeros(2)
        kind = None; vr = None; los = None; sensors = []
        for idx in group:
            d = dets[idx]
            Ri = np.linalg.inv(d.R)
            info += Ri; infz += Ri @ d.z
            sensors.append(d.sensor)
            if d.kind is not None:
                kind = d.kind
            if d.sensor == "radar" and d.vr is not None:
                vr = d.vr
        R = np.linalg.inv(info)
        z = R @ infz
        v_prior = None
        if vr is not None:
            # project the radial speed onto the (assumed forward) LOS from origin
            # we don't know the ego here; approximate LOS by detection bearing in world
            ang = np.arctan2(z[1], z[0]) if np.hypot(*z) > 1 else 0.0
            v_prior = np.array([vr * np.cos(ang), vr * np.sin(ang)])
        out.append(FusedDetection(z, R, kind or "unknown", v_prior, sensors))
    return out


def perceive(suite, ego, objects, gate=9.21):
    """Sensors -> fused object list (positions + covariance + class + v prior)."""
    return fuse(suite.measure(ego, objects), gate=gate)
