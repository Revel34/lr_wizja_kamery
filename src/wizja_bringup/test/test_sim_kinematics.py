import math

import pytest

from wizja_bringup.sim_kinematics import STEER_LIMIT, Limits, RoverPose, swerve_wheels

# Rover 3.0 steer joint positions in base_link (rover30 URDF).
WHEELS = {
    'left_front': (0.48131, 0.37355),
    'right_front': (0.48137, -0.37345),
    'left_rear': (-0.36144, 0.37355),
    'right_rear': (-0.36137, -0.37345),
}


def test_forward_all_wheels_straight_same_speed():
    angles, speeds = swerve_wheels(0.5, 0.0, 0.0, WHEELS)
    assert all(a == pytest.approx(0.0) for a in angles.values())
    assert all(v == pytest.approx(0.5) for v in speeds.values())


def test_backward_keeps_wheels_straight_with_negative_speed():
    angles, speeds = swerve_wheels(-0.5, 0.0, 0.0, WHEELS)
    assert all(a == pytest.approx(0.0) for a in angles.values())
    assert all(v == pytest.approx(-0.5) for v in speeds.values())


@pytest.mark.parametrize('vy', [0.4, -0.4])
def test_sideways_turns_wheels_ninety_degrees(vy):
    angles, speeds = swerve_wheels(0.0, vy, 0.0, WHEELS)
    for name in WHEELS:
        assert abs(angles[name]) == pytest.approx(math.pi / 2)
        assert abs(angles[name]) <= STEER_LIMIT + 1e-9
        assert abs(speeds[name]) == pytest.approx(0.4)


def test_turn_on_the_spot_wheels_tangent_to_circle():
    angles, speeds = swerve_wheels(0.0, 0.0, 1.0, WHEELS)
    for name, (px, py) in WHEELS.items():
        # Wheel direction must be perpendicular to its lever arm.
        dx, dy = math.cos(angles[name]), math.sin(angles[name])
        assert dx * px + dy * py == pytest.approx(0.0, abs=1e-9)
        assert abs(speeds[name]) == pytest.approx(math.hypot(px, py))


def test_reverse_diagonal_flips_angle_and_reverses_speed():
    angles, speeds = swerve_wheels(-0.5, 0.5, 0.0, WHEELS)
    for name in WHEELS:
        # Travel direction is 135 deg; beyond the +/-90 deg joint limit -> -45 deg, driving backwards.
        assert angles[name] == pytest.approx(-math.pi / 4)
        assert speeds[name] == pytest.approx(-math.hypot(0.5, 0.5))


def test_stopped_wheels_keep_last_angle():
    previous = {name: 0.3 for name in WHEELS}
    angles, speeds = swerve_wheels(0.0, 0.0, 0.0, WHEELS, previous)
    assert angles == previous
    assert all(v == 0.0 for v in speeds.values())


def test_pose_integration_straight_line():
    pose = RoverPose(limits=Limits(max_linear_acc=1e9, max_angular_acc=1e9))
    t, dt = 0.0, 0.01
    for _ in range(100):  # 1 s at 0.5 m/s
        pose.command(0.5, 0.0, 0.0, t)
        t += dt
        pose.step(t, dt)
    assert pose.x == pytest.approx(0.5, abs=1e-6)
    assert pose.y == pytest.approx(0.0, abs=1e-9)


def test_pose_integration_follows_heading():
    pose = RoverPose(limits=Limits(max_linear_acc=1e9, max_angular_acc=1e9), yaw=math.pi / 2)
    pose.command(1.0, 0.0, 0.0, 0.0)
    pose.step(0.1, 0.1)
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(0.1)


def test_limits_and_acceleration():
    pose = RoverPose()
    pose.command(5.0, 0.0, 9.0, 0.0)               # clamped to 1 m/s, 1 rad/s
    pose.step(0.1, 0.1)
    assert pose.vx == pytest.approx(0.1)            # 1 m/s^2 * 0.1 s
    assert pose.wz == pytest.approx(0.2)            # 2 rad/s^2 * 0.1 s
    for i in range(2, 40):
        pose.command(5.0, 0.0, 9.0, i * 0.1)
        pose.step(i * 0.1, 0.1)
    assert pose.vx == pytest.approx(1.0)
    assert pose.wz == pytest.approx(1.0)


def test_command_timeout_stops_the_rover():
    pose = RoverPose(limits=Limits(max_linear_acc=1e9, max_angular_acc=1e9))
    pose.command(0.5, 0.0, 0.0, 0.0)
    pose.step(0.1, 0.1)
    assert pose.vx == pytest.approx(0.5)
    pose.step(1.0, 0.1)                             # last command 1.0 s ago > 0.5 s timeout
    assert pose.vx == 0.0 and pose.vy == 0.0 and pose.wz == 0.0
