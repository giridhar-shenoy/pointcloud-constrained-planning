"""Forward kinematics straight from a URDF file (numpy only), for the GIF overlays.

The GIFs print the tool's tilt and draw a marker on the tool, both computed here from recorded
joint positions, never typed. Tilt has the benchmark's definition (pcp_benchmark metrics.cpp,
tiltFromDownDeg): the angle between tool0's z axis and the world's -z axis.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    """URDF rpy: fixed-axis roll, pitch, yaw = Rz(y) @ Ry(p) @ Rx(r)."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def homogeneous(rot, trans):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


class UrdfFk:
    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}  # child link -> joint
        for j in root.findall("joint"):
            origin = j.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
                rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            axis = j.find("axis")
            self.joints[j.find("child").get("link")] = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "origin": homogeneous(rpy_matrix(*rpy), xyz),
                "axis": [float(v) for v in axis.get("xyz").split()] if axis is not None else None,
            }

    def chain(self, link):
        out = []
        while link in self.joints:
            out.append(self.joints[link])
            link = self.joints[link]["parent"]
        return list(reversed(out))

    def pose(self, link, positions):
        """4x4 pose of `link` in the URDF root frame; `positions` maps joint name -> value."""
        t = np.eye(4)
        for j in self.chain(link):
            t = t @ j["origin"]
            if j["type"] in ("revolute", "continuous"):
                t = t @ homogeneous(axis_angle(j["axis"], positions[j["name"]]), [0, 0, 0])
            elif j["type"] == "prismatic":
                t = t @ homogeneous(np.eye(3), np.asarray(j["axis"]) * positions[j["name"]])
        return t


def tilt_deg(pose):
    """Angle (deg) between the tool z axis and world -z, as tiltFromDownDeg in metrics.cpp."""
    c = max(-1.0, min(1.0, -pose[2, 2]))
    return math.degrees(math.acos(c))


def densify(path, max_step=0.01):
    """pcp_benchmark::densify: linear joint steps of at most `max_step` rad on every joint."""
    dense = [list(path[0])]
    for a, b in zip(path[:-1], path[1:], strict=True):
        a = np.asarray(a)
        b = np.asarray(b)
        steps = max(1, math.ceil(float(np.max(np.abs(b - a))) / max_step))
        for s in range(1, steps + 1):
            dense.append(list(a + (b - a) * (s / steps)))
    return dense
