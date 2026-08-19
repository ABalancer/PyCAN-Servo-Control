from CanHelpers import *
from MotorInterface import MotorInterface


class MotorSystem:
    def __init__(self, device_dictionary, bus_type, usb_port, bitrate):
        self._bus: CanBus | None = None
        self._system_enabled: bool = False

        try:
            self._bus = CanBus(bus_type, usb_port, bitrate)

        except Exception as e:
            print(f"CAN bus open failed ({e}).")
            exit("Exiting Program.")

        if self._bus:
            self._motors: dict[str, MotorInterface] = {}
            for axis_key in list(device_dictionary.keys()):
                axis = MotorInterface(self._bus, axis_key, device_dictionary[axis_key])
                self._motors[axis_key] = axis

            self._active_axis_key: str = list(self._motors.keys())[-1]
            self._active_axis: MotorInterface = self._motors[self._active_axis_key]
            self._set_position: dict[str, float] = {key: 0.0 for key in self._motors.keys()}

            # Check enabled state
            enabled_list = []
            for axis_key in device_dictionary:
                enabled_state = self._motors[axis_key].get_last_motor_enabled_state()
                if enabled_state:
                    enabled_list.append(enabled_state)

            if len(enabled_list) > 0:
                self._system_enabled = all(enabled_list)  # if all values are true, system is enabled


    def close_can(self):
        if self._bus:
            self._bus.close()

    def get_system_keys(self) -> list[str]:
        return list(self._motors.keys())

    def set_active_axis(self, key: str):
        self._active_axis = self._motors[key]
        self._active_axis_key = key

    def get_active_axis_name(self) -> str:
        return self._active_axis_key

    def get_active_axis_object(self) -> MotorInterface:
        return self._active_axis

    def get_active_axis_arbitration_id(self) -> int:
        return self._motors[self._active_axis_key].get_can_id()

    def set_axis_position(self, position: float, speed: float, acceleration: float,
                          key: str | None = None):
        key = key if key else self._active_axis_key
        self._motors[key].cmd_set_absolute_linear_position(position, speed, acceleration)
        self._set_position[key] = position

    def jog_axis(self, change_in_position: float, speed: float, acceleration: float,
                 key: str | None = None):
        key = key if key else self._active_axis_key
        self._set_position[key] += change_in_position
        self.set_axis_position(self._set_position[key], speed, acceleration, key)

    def check_axis_enabled(self, key: str | None = None) -> bool | None:
        if key is None:
            key = self._active_axis_key
        return self._motors[key].get_last_motor_enabled_state()

    def check_axis_requires_homing(self, key: str | None = None) -> bool:
        if key is None:
            key = self._active_axis_key
        return self._motors[key].check_requires_homing()

    def check_axis_homed(self, key: str | None = None) -> bool:
        if key is None:
            key = self._active_axis_key
        return self._motors[key].check_homed()

    def home_axis(self, key: str | None = None):
        if key is None:
            key = self._active_axis_key
        self._motors[key].cmd_start_homing()
        self._set_position[key] = 0.0

    def zero_axis(self, key: str | None = None):
        if key is None:
            key = self._active_axis_key
        self._motors[key].cmd_set_zero()

    def stop_axis(self, acceleration: float, key: str | None = None):
        key = key if key else self._active_axis_key
        if acceleration > 0:
            self._motors[key].cmd_soft_stop(acceleration)
        else:
            self._motors[key].cmd_emergency_stop()

    def check_axis_move_allowed(self, key: str | None = None) -> bool:
        if key is None:
            key = self._active_axis_key
        return self._motors[key].check_move_allowed()

    def toggle_motor_enable(self) -> bool:
        if self._system_enabled:
            self._system_enabled = False
        else:
            self._system_enabled = True

        for motor in self._motors:
            self._motors[motor].cmd_enable_motors(self._system_enabled)

        time.sleep(0.2)

        for motor in self._motors:
            enable_state = self._motors[motor].cmd_check_motors_enabled()
            if enable_state:
                if self._system_enabled != enable_state:
                    print("%02X: error does not have enable state: %s" % (self._motors[motor].get_can_id(), self._system_enabled))
                    return not self._system_enabled

        return self._system_enabled

    def get_system_enabled(self) -> bool:
        return self._system_enabled

    def get_set_position(self, key: str | None = None) -> float:
        key = key if key else self._active_axis_key
        return self._set_position[key]
