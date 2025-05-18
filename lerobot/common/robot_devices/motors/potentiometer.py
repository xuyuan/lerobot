import enum
from threading import Thread
import serial
import time
import numpy as np
from lerobot.common.robot_devices.motors.configs import PotentiometerBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError


# The following bounds define the lower and upper joints range (after calibration).
# For joints in degree (i.e. revolute joints), their nominal range is [-180, 180] degrees
# which corresponds to a half rotation on the left and half rotation on the right.
# Some joints might require higher range, so we allow up to [-270, 270] degrees until
# an error is raised.
LOWER_BOUND_DEGREE = -270
UPPER_BOUND_DEGREE = 270
# For joints in percentage (i.e. joints that move linearly like the prismatic joint of a gripper),
# their nominal range is [0, 100] %. For instance, for Aloha gripper, 0% is fully
# closed, and 100% is fully open. To account for slight calibration issue, we allow up to
# [-10, 110] until an error is raised.
LOWER_BOUND_LINEAR = -10
UPPER_BOUND_LINEAR = 110

HALF_TURN_DEGREE = 180


class CalibrationMode(enum.Enum):
    # Joints with rotational motions are expressed in degrees in nominal range of [-180, 180]
    DEGREE = 0
    # Joints with linear motions (like gripper of Aloha) are expressed in nominal range of [0, 100]
    LINEAR = 1


class JointOutOfRangeError(Exception):
    def __init__(self, message="Joint is out of range"):
        self.message = message
        super().__init__(self.message)


class PotentiometerBus(object):
    """
    PotentiometerBus is a class that represents a bus of potentiometers.
    It is used to read the values of the potentiometers and apply calibration.
    """

    def __init__(self,
                 config: PotentiometerBusConfig):
        self.port = config.port
        self.motors = config.motors
        self.mock = config.mock

        self.calibration = None
        self.connection = None
        self.thread = None
        self.last_readline = None

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]
    
    def set_calibration(self, calibration: dict[str, list]):
        self.calibration = calibration

    def connect(self):
        if self.connection is None:
            self.connection = serial.Serial(self.port, 115200, timeout=1)
            self.thread = Thread(target=self.main)
            self.thread.daemon = True
            self.thread.start()
        
        while self.last_readline is None:
            print("Waiting for potentiometer data...")
            time.sleep(1.0)

    def disconnect(self):
        if not self.connection:
            raise RobotDeviceNotConnectedError(
                f"PotentiometerBus({self.port}) is not connected. Try running `motors_bus.connect()` first."
            )

        self.connection = None

    def main(self):
        if self.connection is None or not self.connection.is_open:
            raise RuntimeError("Connection not established. Call connect() first.")

        # Read data from the potentiometers
        while self.connection is not None and self.connection.is_open:
            l = self.connection.readline()
            if l:
                self.last_readline = l
            

    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        pass

    def read(self, data_name, motor_names: str | list[str] | None = None):
        if data_name == "Torque_Enable":
            return np.array([0] * len(self.motors))
        elif data_name == "Present_Position":
            values = str(self.last_readline, 'ascii').split(',')
            values = [values[i] for i in self.motor_indices]
            values = np.array([int(value)*4 for value in values])
            if self.calibration is not None:
                values[-1] = max(3, 4 - values[-1])
                values = self.apply_calibration(values, motor_names)
            return values
            
        print(f"Reading {data_name} from {motor_names}")

    def apply_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        if motor_names is None:
            motor_names = self.motor_names
        # Convert from unsigned int32 original range [0, 2**32] to signed float32 range
        values = values.astype(np.float32)

        for i, name in enumerate(motor_names):
            calib_idx = self.calibration["motor_names"].index(name)
            calib_mode = self.calibration["calib_mode"][calib_idx]

            if CalibrationMode[calib_mode] == CalibrationMode.DEGREE:
                drive_mode = self.calibration["drive_mode"][calib_idx]
                homing_offset = self.calibration["homing_offset"][calib_idx]
                _, model = self.motors[name]
                resolution = 4096

                # Update direction of rotation of the motor to match between leader and follower.
                # In fact, the motor of the leader for a given joint can be assembled in an
                # opposite direction in term of rotation than the motor of the follower on the same joint.
                if drive_mode:
                    values[i] *= -1

                # Convert from range [-2**31, 2**31[ to
                # nominal range ]-resolution, resolution[ (e.g. ]-2048, 2048[)
                values[i] += homing_offset

                # Convert from range ]-resolution, resolution[ to
                # universal float32 centered degree range ]-180, 180[
                values[i] = values[i] / (resolution // 2) * HALF_TURN_DEGREE

                if (values[i] < LOWER_BOUND_DEGREE) or (values[i] > UPPER_BOUND_DEGREE):
                    raise JointOutOfRangeError(
                        f"Wrong motor position range detected for {name}. "
                        f"Expected to be in nominal range of [-{HALF_TURN_DEGREE}, {HALF_TURN_DEGREE}] degrees (a full rotation), "
                        f"with a maximum range of [{LOWER_BOUND_DEGREE}, {UPPER_BOUND_DEGREE}] degrees to account for joints that can rotate a bit more, "
                        f"but present value is {values[i]} degree. "
                        "This might be due to a cable connection issue creating an artificial 360 degrees jump in motor values. "
                        "You need to recalibrate by running: `python lerobot/scripts/control_robot.py calibrate`"
                    )

            elif CalibrationMode[calib_mode] == CalibrationMode.LINEAR:
                start_pos = self.calibration["start_pos"][calib_idx]
                end_pos = self.calibration["end_pos"][calib_idx]

                # Rescale the present position to a nominal range [0, 100] %,
                # useful for joints with linear motions like Aloha gripper
                values[i] = (values[i] - start_pos) / (end_pos - start_pos) * 100

                if (values[i] < LOWER_BOUND_LINEAR) or (values[i] > UPPER_BOUND_LINEAR):
                    raise JointOutOfRangeError(
                        f"Wrong motor position range detected for {name}. "
                        f"Expected to be in nominal range of [0, 100] % (a full linear translation), "
                        f"with a maximum range of [{LOWER_BOUND_LINEAR}, {UPPER_BOUND_LINEAR}] % to account for some imprecision during calibration, "
                        f"but present value is {values[i]} %. "
                        "This might be due to a cable connection issue creating an artificial jump in motor values. "
                        "You need to recalibrate by running: `python lerobot/scripts/control_robot.py calibrate`"
                    )

        return values