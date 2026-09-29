"""
Tests for the DVL mounting rotation.

The legacy_* functions reproduce the DOWN/FORWARD handling of DvlDriver.handle_velocity
from before the general mounting rotation, so the presets can be checked against it exactly.
"""

import json
import math
from typing import Any, Dict, List, Optional, Tuple

import pytest

from dvl import DVL_CUSTOM, DVL_DOWN, DVL_FORWARD, DvlDriver, MessageType
from mount import MountAngles, MountRotation, validate_angle_deg

Call = Tuple[str, Any]

VELOCITY_REPORTS = [
    {"vx": 0.31, "vy": -0.12, "vz": 0.057, "altitude": 2.43, "fom": 0.013, "time": 72.5},
    {"vx": -1.7, "vy": 0.9, "vz": -0.33, "altitude": 11.2, "fom": 0.2, "time": 250.0},
    {"vx": 0.0, "vy": 0.0, "vz": 0.0, "altitude": 0.5, "fom": 0.5, "time": 1.0},
]
# (current_attitude, last_attitude) in degrees, as reported by the DVL's position_local
ATTITUDES = [((1.5, -2.25, 179.0), (1.25, -2.0, 178.5)), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))]


class FakeMav:
    def __init__(self) -> None:
        self.calls: List[Call] = []

    def send_rangefinder(self, distance: float) -> None:
        self.calls.append(("rangefinder", distance))

    def send_vision(self, position_deltas, rotation_deltas, confidence=100, dt=125000) -> None:
        self.calls.append(("vision", (list(position_deltas), list(rotation_deltas), confidence, dt)))

    def send_vision_speed_estimate(self, speed_estimates) -> None:
        self.calls.append(("speed", list(speed_estimates)))


def legacy_handle_velocity(
    orientation: int, should_send: MessageType, data: Dict[str, Any], current_attitude, last_attitude
) -> List[Call]:
    """
    handle_velocity as it was with the DVL_DOWN/DVL_FORWARD presets (rangefinder enabled)
    """
    calls: List[Call] = []
    vx, vy, vz, alt, fom = data["vx"], data["vy"], data["vz"], data["altitude"], data["fom"]
    dt = data["time"] / 1000
    dx = dt * vx
    dy = dt * vy
    dz = dt * vz
    _fom_max = 0.4
    confidence = 100 * (1 - min(_fom_max, fom) / _fom_max)
    if alt > 0.05:
        calls.append(("rangefinder", alt))
    if should_send == MessageType.POSITION_DELTA:
        dRoll, dPitch, dYaw = [current - last for (current, last) in zip(current_attitude, last_attitude)]
        if orientation == DVL_DOWN:
            position_delta = [dx, dy, dz]
            attitude_delta = [dRoll, dPitch, dYaw]
        else:
            position_delta = [dz, dy, -dx]
            attitude_delta = [dYaw, dPitch, -dRoll]
        calls.append(("vision", (position_delta, attitude_delta, confidence, data["time"] * 1e3)))
    elif should_send == MessageType.SPEED_ESTIMATE:
        velocity = [vx, vy, vz] if orientation == DVL_DOWN else [vz, vy, -vx]
        calls.append(("speed", velocity))
    return calls


def make_driver(tmp_path, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> DvlDriver:
    driver = DvlDriver()
    driver.settings_path = str(tmp_path / "settings.json")
    driver.mav = FakeMav()
    driver.rangefinder = True
    driver.mount_pitch_deg = pitch_deg
    driver.mount_roll_deg = roll_deg
    return driver


def run_driver(
    driver: DvlDriver, should_send: MessageType, data: Dict[str, Any], current_attitude, last_attitude
) -> List[Call]:
    driver.should_send = should_send
    driver.current_attitude = current_attitude
    driver.last_attitude = last_attitude
    driver.mav.calls = []
    driver.handle_velocity({**data, "velocity_valid": True})
    return driver.mav.calls


def without_rangefinder(calls: List[Call]) -> List[Call]:
    return [call for call in calls if call[0] != "rangefinder"]


@pytest.mark.parametrize("should_send", [MessageType.POSITION_DELTA, MessageType.SPEED_ESTIMATE])
@pytest.mark.parametrize("data", VELOCITY_REPORTS)
@pytest.mark.parametrize("attitudes", ATTITUDES)
def test_zero_pitch_matches_legacy_down_exactly(tmp_path, should_send, data, attitudes):
    driver = make_driver(tmp_path, pitch_deg=0.0)
    expected = legacy_handle_velocity(DVL_DOWN, should_send, data, *attitudes)
    assert run_driver(driver, should_send, data, *attitudes) == expected


@pytest.mark.parametrize("should_send", [MessageType.POSITION_DELTA, MessageType.SPEED_ESTIMATE])
@pytest.mark.parametrize("data", VELOCITY_REPORTS)
@pytest.mark.parametrize("attitudes", ATTITUDES)
def test_ninety_pitch_matches_legacy_forward_exactly(tmp_path, should_send, data, attitudes):
    driver = make_driver(tmp_path, pitch_deg=90.0)
    expected = legacy_handle_velocity(DVL_FORWARD, should_send, data, *attitudes)
    calls = run_driver(driver, should_send, data, *attitudes)
    # Velocity and attitude deltas are bit-identical to the legacy FORWARD mapping
    assert without_rangefinder(calls) == without_rangefinder(expected)
    # Deliberate change: a forward-looking DVL no longer reports its forward range as a
    # downward rangefinder, since its projection on the vehicle's down axis is 0
    assert [call for call in calls if call[0] == "rangefinder"] == []


@pytest.mark.parametrize("data", VELOCITY_REPORTS)
@pytest.mark.parametrize("attitudes", ATTITUDES)
def test_thirty_pitch(tmp_path, data, attitudes):
    c, s = math.sqrt(3) / 2, 0.5
    vx, vy, vz, alt = data["vx"], data["vy"], data["vz"], data["altitude"]
    dt = data["time"] / 1000
    rates = [current - last for (current, last) in zip(*attitudes)]
    # Nose-up tilt about the vehicle y axis: DVL x leans up, DVL z (beams) leans forward
    expected_velocity = [c * vx + s * vz, vy, -s * vx + c * vz]
    expected_attitude = [c * rates[0] + s * rates[2], rates[1], -s * rates[0] + c * rates[2]]

    driver = make_driver(tmp_path, pitch_deg=30.0)

    calls = run_driver(driver, MessageType.SPEED_ESTIMATE, data, *attitudes)
    assert calls[0] == ("rangefinder", pytest.approx(alt * c))
    assert calls[1] == ("speed", pytest.approx(expected_velocity))

    calls = run_driver(driver, MessageType.POSITION_DELTA, data, *attitudes)
    assert calls[0] == ("rangefinder", pytest.approx(alt * c))
    position_delta, attitude_delta, _, _ = calls[1][1]
    assert position_delta == pytest.approx([dt * v for v in expected_velocity])
    assert attitude_delta == pytest.approx(expected_attitude)


def test_sign_convention():
    # Positive pitch = DVL nose-up: the DVL's x axis points forward and up (negative z) in the vehicle frame
    nose = MountRotation(MountAngles(pitch_deg=30.0)).rotate([1.0, 0.0, 0.0])
    assert nose[2] < 0 < nose[0]
    # At 90 deg the transducer face (DVL +z) points straight forward
    assert MountRotation(MountAngles(pitch_deg=90.0)).rotate([0.0, 0.0, 1.0]) == [1.0, 0.0, 0.0]
    # Positive roll = right side down: the DVL's y axis points down
    assert MountRotation(MountAngles(roll_deg=90.0)).rotate([0.0, 1.0, 0.0]) == [0.0, 0.0, 1.0]


def test_rotation_preserves_length_and_composes_pitch_then_roll():
    rotation = MountRotation(MountAngles(pitch_deg=-20.0, roll_deg=15.0))
    vector = [0.4, -1.1, 0.7]
    assert math.hypot(*rotation.rotate(vector)) == pytest.approx(math.hypot(*vector))
    pitch_only = MountRotation(MountAngles(pitch_deg=-20.0))
    roll_only = MountRotation(MountAngles(roll_deg=15.0))
    assert rotation.rotate(vector) == pytest.approx(pitch_only.rotate(roll_only.rotate(vector)))


def test_looking_up_does_not_send_rangefinder(tmp_path):
    driver = make_driver(tmp_path, pitch_deg=180.0)
    calls = run_driver(driver, MessageType.SPEED_ESTIMATE, VELOCITY_REPORTS[0], *ATTITUDES[0])
    assert [call[0] for call in calls] == ["speed"]


@pytest.mark.parametrize("angle", [float("nan"), float("inf"), 180.5, -181.0, "abc"])
def test_invalid_angles_rejected(tmp_path, angle):
    with pytest.raises(ValueError):
        validate_angle_deg(angle)
    driver = make_driver(tmp_path, pitch_deg=10.0, roll_deg=5.0)
    assert not driver.set_mount_angles(pitch_deg=angle)
    assert not driver.set_mount_angles(roll_deg=angle)
    assert (driver.mount_pitch_deg, driver.mount_roll_deg) == (10.0, 5.0)


def test_settings_round_trip_and_presets(tmp_path):
    driver = make_driver(tmp_path)
    assert driver.set_mount_angles(pitch_deg=30.0)
    assert driver.set_mount_angles(roll_deg=-5.0)
    assert driver.orientation == DVL_CUSTOM

    loaded = make_driver(tmp_path)
    loaded.load_settings()
    assert (loaded.mount_pitch_deg, loaded.mount_roll_deg) == (30.0, -5.0)
    assert loaded.get_status()["mount_pitch_deg"] == 30.0

    assert loaded.set_orientation(DVL_FORWARD)
    assert (loaded.mount_pitch_deg, loaded.mount_roll_deg, loaded.orientation) == (90.0, 0.0, DVL_FORWARD)
    assert loaded.set_orientation(DVL_DOWN)
    assert (loaded.mount_pitch_deg, loaded.orientation) == (0.0, DVL_DOWN)
    assert not loaded.set_orientation(3)


@pytest.mark.parametrize("orientation, pitch", [(DVL_DOWN, 0.0), (DVL_FORWARD, 90.0)])
def test_legacy_orientation_setting_is_migrated(tmp_path, orientation, pitch):
    (tmp_path / "settings.json").write_text(json.dumps({"enabled": True, "orientation": orientation}))
    driver = make_driver(tmp_path, pitch_deg=45.0)
    driver.load_settings()
    assert (driver.mount_pitch_deg, driver.mount_roll_deg, driver.orientation) == (pitch, 0.0, orientation)


class FakeGimbal:
    def __init__(self) -> None:
        self.requested: List[float] = []

    def angles_at(self, timestamp: float) -> MountAngles:
        self.requested.append(timestamp)
        return MountAngles(pitch_deg=90.0, timestamp=timestamp)


@pytest.mark.parametrize("time_of_validity", [None, 1_700_000_000_123_456])
def test_live_angle_source_overrides_fixed_angles(tmp_path, time_of_validity: Optional[int]):
    driver = make_driver(tmp_path, pitch_deg=0.0)
    driver.mount_angle_source = FakeGimbal()
    data = dict(VELOCITY_REPORTS[0])
    if time_of_validity is not None:
        data["time_of_validity"] = time_of_validity
    calls = run_driver(driver, MessageType.SPEED_ESTIMATE, data, *ATTITUDES[0])
    assert calls == [("speed", [data["vz"], data["vy"], -data["vx"]])]
    if time_of_validity is not None:
        assert driver.mount_angle_source.requested == [pytest.approx(time_of_validity * 1e-6)]
