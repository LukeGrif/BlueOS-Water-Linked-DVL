"""
Mounting rotation between the DVL and the vehicle frame.

Both frames are FRD (x forward, y right, z down). The DVL frame is the one Water Linked
reports data in: z points out of the transducer face, x points towards the LED side.

Sign convention (right-handed rotations of the DVL relative to the vehicle frame):

* ``pitch_deg``: positive = DVL tilted nose-up relative to the frame, i.e. a rotation
  about the vehicle +y axis that lifts the DVL's +x axis towards the vehicle's -z (up).
  0 deg is the DOWN preset (transducers facing down), 90 deg is the FORWARD preset
  (transducers facing forward, DVL +x pointing up).
* ``roll_deg``: positive = DVL rolled right side down, i.e. a rotation about the DVL +x axis.

A vector measured in the DVL frame is expressed in the vehicle frame with
``v_vehicle = R @ v_dvl``, where ``R = Ry(pitch) @ Rx(roll)`` (pitch applied first, then
roll about the already pitched DVL x axis, as in the usual yaw-pitch-roll sequence).

The rotation is always built from a ``MountAngles`` sample returned by a
``MountAngleSource`` for the time of the measurement. Today the only source is the fixed
angle from the settings; a gimbal can later provide live, timestamped samples through the
same interface without touching the code that applies the rotation.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Protocol, Sequence, Tuple

Vector = Sequence[float]
Matrix = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]

# Presets for the legacy "orientation" setting
DOWN_PITCH_DEG = 0.0
FORWARD_PITCH_DEG = 90.0


@dataclass(frozen=True)
class MountAngles:
    """
    DVL mounting angles, in degrees, following the sign convention in the module docstring.
    timestamp is the time (seconds, same clock as the DVL measurements) the angles were valid at,
    or None for a fixed mount.
    """

    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    timestamp: Optional[float] = None


class MountAngleSource(Protocol):
    """
    Anything that can tell the mounting angles at a given time, e.g. a gimbal feed that
    interpolates between its timestamped samples.
    """

    def angles_at(self, timestamp: float) -> MountAngles: ...


def validate_angle_deg(angle: float) -> float:
    """
    Returns angle as a float, raising ValueError if it is not finite or outside [-180, 180]
    """
    angle = float(angle)
    if not math.isfinite(angle) or not -180.0 <= angle <= 180.0:
        raise ValueError(f"angle must be a finite number of degrees in [-180, 180], got {angle}")
    return angle


def _cos_sin_deg(angle_deg: float) -> Tuple[float, float]:
    """
    cos and sin of an angle in degrees, exact at multiples of 90 deg so the presets
    reproduce the plain axis swaps bit for bit (math.cos(pi / 2) is 6e-17, not 0)
    """
    if not angle_deg % 90:
        quadrant = int(angle_deg // 90) % 4
        return ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))[quadrant]
    angle = math.radians(angle_deg)
    return math.cos(angle), math.sin(angle)


class MountRotation:
    """
    Rotation from the DVL frame to the vehicle frame for one set of mounting angles
    """

    def __init__(self, angles: MountAngles) -> None:
        self.angles = angles
        cp, sp = _cos_sin_deg(angles.pitch_deg)
        cr, sr = _cos_sin_deg(angles.roll_deg)
        # Ry(pitch) @ Rx(roll)
        self.matrix: Matrix = (
            (cp, sp * sr, sp * cr),
            (0.0, cr, -sr),
            (-sp, cp * sr, cp * cr),
        )

    def rotate(self, vector: Vector) -> List[float]:
        """
        Expresses a DVL-frame vector (velocity, displacement or small rotation) in the vehicle frame
        """
        return [row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2] for row in self.matrix]

    def range_to_vehicle_down(self, distance: float) -> float:
        """
        Projects a distance measured along the DVL z axis (the DVL "altitude") onto the vehicle's
        down axis. Negative or ~0 when the DVL is not looking downwards.
        """
        return distance * self.matrix[2][2]
