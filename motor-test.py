import glob
import math
import time

from robstride_dynamics import (
    Motor,
    RobstrideBus,
    ParameterType,
    CommunicationType,
)


def find_usbmodem_channel() -> str:
    """Find any device with 'usbmodem' in the name."""
    devices = glob.glob("/dev/tty.usbmodem*")
    if not devices:
        raise RuntimeError("No usbmodem device found")
    return devices[0]


CHANNEL = find_usbmodem_channel()
INTERFACE = "slcan"
TEST_POSITIONS = (0.0, 1.0, 0.0)
HOLD_SECS = 1.0
KP = 100.0
KD = 8.0
PARAMETERS_TO_READ = tuple(
    value
    for name, value in vars(ParameterType).items()
    if not name.startswith("__") and isinstance(value, tuple)
)
PARAMETERS_TO_SET = (
    (ParameterType.MODE, 0),
    (ParameterType.CURRENT_KP, 0.15),
    (ParameterType.CURRENT_KI, 0.015),
    (ParameterType.CURRENT_FILTER_GAIN, 1.0),
    (ParameterType.TORQUE_LIMIT, 120.0),
    (ParameterType.VELOCITY_LIMIT, 0.0),
    (ParameterType.CURRENT_LIMIT, 30.0),
    (ParameterType.POSITION_KP, 40.0),
    (ParameterType.VELOCITY_KP, 6.0),
    (ParameterType.VELOCITY_KI, 0.02),
    (ParameterType.VELOCITY_FILTER_GAIN, 0.01),
    (ParameterType.PP_VELOCITY_MAX, 0.0),
    (ParameterType.PP_ACCELERATION_TARGET, 0.0),
    (ParameterType.EPSCAN_TIME, 0),
)


def configure_motor_params(bus: RobstrideBus) -> None:
    names = list(bus.motors)
    for name in names:
        for param, value in PARAMETERS_TO_SET:
            bus.write(name, param, value)
        bus.transmit(
            CommunicationType.SAVE_PARAMETERS,
            bus.host_id,
            bus.motors[name].id,
        )
        time.sleep(0.1)
        bus.enable(name)
        bus.set_zero_position(name)
        time.sleep(0.1)


def print_motor_parameters(bus: RobstrideBus) -> None:
    names = list(bus.motors)
    for name in names:
        bus.enable(name)
        for param in PARAMETERS_TO_READ:
            value = bus.read(name, param)
            print(f"{name}.{param[2]} = {value}")


def send_test_motion(bus: RobstrideBus, positions: tuple[float, ...]) -> None:
    names = list(bus.motors)
    for name in names:
        bus.enable(name)

    try:
        for target in positions:
            per_motor_durations: dict[str, float] = {}
            for name in names:
                start = time.perf_counter()
                bus.write_operation_frame(
                    motor=name,
                    position=target,
                    kp=KP,
                    kd=KD,
                    velocity=0.0,
                    torque=0.0,
                )
                per_motor_durations[name] = (time.perf_counter() - start) * 1000.0
            time.sleep(HOLD_SECS)
            for name in names:
                status = bus.read_operation_frame(name)
                print(
                    f"{name}: position={status[0]:.3f} rad, velocity={status[1]:.3f} rad/s, "
                    f"elapsed={per_motor_durations[name]:.1f} ms"
                )
    finally:
        for name in names:
            bus.disable(name)


def run_sine_wave(
    bus: RobstrideBus,
    amplitude: float = math.pi,
    frequency_hz: float = 0.6,
    dt: float = 0.005,
    cycles: int = 1,
) -> None:
    names = list(bus.motors)
    baselines: dict[str, float] = {}
    for name in names:
        bus.enable(name)
        baselines[name] = bus.read(name, ParameterType.MECHANICAL_POSITION)
        print(
            f"{name}: baseline position={baselines[name]:.3f} rad "
        )

    total_samples = int((cycles / frequency_hz) / dt)
    t0 = time.perf_counter()
    loop_start = time.perf_counter()

    for sample in range(total_samples):
        t = time.perf_counter()
        angle = amplitude * math.sin(2 * math.pi * frequency_hz * (t - t0))
        for name in names:
            bus.write_operation_frame(
                motor=name,
                position=baselines[name] + angle,
                kp=KP,
                kd=KD,
                velocity=0.0,
                torque=0.0,
            )
            bus.read_operation_frame(name)
        time.sleep(max(dt - (time.perf_counter() - t), 0))

    loop_duration_ms = (time.perf_counter() - loop_start) * 1000.0
    print(f"Sine wave loop completed in {loop_duration_ms:.1f} ms")

    for name in names:
        bus.disable(name)


def main() -> None:
    motors = {
        #"left_arm_elbow": Motor(id=67, model="rs-02"),
        # "right_ankle": Motor(id=52, model="rs-02"),
        # "right_ankle_lower": Motor(id=53, model="rs-02"),
        "left_knee": Motor(id=35, model="rs-03")
        }

    bus = RobstrideBus(channel=CHANNEL, motors=motors, interface=INTERFACE)
    bus.connect()
    try:
        configure_motor_params(bus)
        print_motor_parameters(bus)
        # send_test_motion(bus, positions=TEST_POSITIONS)
        run_sine_wave(bus)
    finally:
        time.sleep(0.1)
        bus.disconnect()


if __name__ == "__main__":
    main()

