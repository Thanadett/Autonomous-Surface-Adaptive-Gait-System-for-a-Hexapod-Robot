import math

from hexapod_state_estimation.attitude import ComplementaryFilter

G = 9.80665


def _accel(roll: float, pitch: float):
    # specific force of gravity in the body frame (R^T (0, 0, g)) for R = Rz Ry(pitch) Rx(roll)
    return (-G * math.sin(pitch), G * math.sin(roll) * math.cos(pitch), G * math.cos(roll) * math.cos(pitch))


def test_static_tilt_is_recovered_with_sign_conventions() -> None:
    f = ComplementaryFilter()
    for k in range(300):
        roll, pitch = f.update(k * 0.01, (0.0, 0.0, 0.0), _accel(math.radians(10), math.radians(-15)))
    assert abs(math.degrees(roll) - 10) < 1e-6
    assert abs(math.degrees(pitch) + 15) < 1e-6   # nose up = negative pitch (REP-103)


def test_gyro_carries_fast_motion_and_accel_noise_is_filtered() -> None:
    f = ComplementaryFilter(tau_s=0.5)
    f.update(0.0, (0, 0, 0), _accel(0, 0))
    rate = math.radians(20)  # 20 deg/s roll for 0.5 s
    for k in range(1, 51):
        t = k * 0.01
        f.update(t, (rate, 0, 0), _accel(rate * t, 0))
    assert abs(math.degrees(f.roll) - 10) < 0.3
    # a 1 m/s^2 forward acceleration spike for 0.1 s (5.8 deg on the accelerometer alone)
    # moves the estimate by only ~1 deg
    for k in range(51, 61):
        ax, ay, az = _accel(rate * 0.5, 0)
        f.update(k * 0.01, (0, 0, 0), (ax + 1.0, ay, az))
    assert abs(math.degrees(f.pitch)) < 1.5
