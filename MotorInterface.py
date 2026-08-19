from Conversions import *
from CanHelpers import *
import time
from concurrent.futures import Future


def future_helper(future:Future, timeout: float) -> Future | None:
    try:
        return future.result(timeout=timeout)
    except TimeoutError:
        return None


# ── Pending reply: wraps a Future keyed by expected response code ─────────────
class _PendingReply:
    def __init__(self, response_code: int, timeout: float = 5.0):
        self.response_code = response_code
        self.future: Future = Future()
        self.deadline = time.monotonic() + timeout

    @property
    def expired(self) -> bool:
        return time.monotonic() > self.deadline


@dataclass
class MotorState:
    encoder_position_deg: float = 0.0
    speed_rpm: int = 0
    enabled: bool | None = None
    homed: bool | None = None
    end_limit_hit: bool = False


class MotorInterface:
    def __init__(self, bus: CanBus, axis_name: str, axis_dictionary: dict[str, dict]):
        self._bus = bus
        self._axis_name: str = axis_name
        self._motor_states: dict[int, MotorState] = {}
        self._pending_lock = threading.Lock()
        self._pending: list[_PendingReply] = []

        self._target_position: float= 0.0
        self._average_encoder_position: float = 0.0

        self._require_homing: bool = True
        self._min_position: float | None = None
        self._max_position: float | None = None
        self._max_rpm: int = 2000
        self._max_mks_acceleration: int = 220
        self._mm_per_revolution: float | int | None = None
        self._can_id: int = 0xFF
        self._node_ids: list[int] = []
        self._counts_per_revolution: int = 16384
        self._relative_command: bool = False

        self._telemetry_callbacks: dict[int, list[Callable]] = {}

        self._process_dictionary(axis_dictionary)

        for node in self._node_ids:
            self._motor_states[node] = MotorState()
            self._motor_states[node].homed = False if self._require_homing else None
            self._bus.register(node, self._on_frame)
            # self.cmd_configure_response(1, 1, node)
            self.cmd_check_foc_work_mode(node)
            self._motor_states[node].enabled = self._cmd_check_motor_enabled(node)
            self.cmd_subscribe(0x31, 100, arbitration_id=node) # position updates

        # assign group id if necessary
        self._number_of_nodes = len(self._node_ids)
        if self._number_of_nodes > 1:
            if self._can_id != self._node_ids[0]:  # may not be necessary
                self.cmd_set_group_ids()

    def _process_dictionary(self, axis_dictionary):
        ids = axis_dictionary.get("ID")
        self._can_id = ids.get("group")
        nodes = ids.get("nodes")
        if nodes is not None:
            self._node_ids = ids.get("nodes")
        else:
            self._node_ids = [self._can_id]

        limits = axis_dictionary.get("limits")
        if limits.get("require_homing") is not None:
            self._require_homing = limits.get("require_homing")

        if limits.get("min_position") is not None:
            self._min_position = limits.get("min_position")

        if limits.get("max_position") is not None:
            self._max_position = limits.get("max_position")

        max_linear_velocity: float | None = None
        if limits.get("velocity") is not None:
            max_linear_velocity = limits.get("velocity")

        linear_acceleration: float | None = None
        if limits.get("acceleration") is not None:
            linear_acceleration = limits.get("acceleration")

        if axis_dictionary.get("encoder_counts") is not None:
            self._counts_per_revolution = axis_dictionary.get("encoder_counts")

        if axis_dictionary.get("mm_per_revolution") is not None:
            self._mm_per_revolution = axis_dictionary.get("mm_per_revolution")
            # system assumed to be rotational with limits assumed degrees if mm_per_revolution is None
            # otherwise given limits are assumed to be distance, with minimum and maximum rescaled
            if self._min_position is not None and self._mm_per_revolution is not None:
                self._min_position = linear_position_to_position_degrees(self._min_position, self._mm_per_revolution)
            if self._max_position is not None and self._mm_per_revolution is not None:
                self._max_position = linear_position_to_position_degrees(self._max_position, self._mm_per_revolution)
            if max_linear_velocity and self._mm_per_revolution:
                self._max_rpm = linear_velocity_to_rpm(max_linear_velocity, self._mm_per_revolution)
            if linear_acceleration and self._mm_per_revolution:
                self._max_mks_acceleration = linear_acceleration_to_byte(linear_acceleration, self._mm_per_revolution)

    # ── Frame dispatcher ──────────────────────────────────────────────────────
    def _on_frame(self, frame: Frame):
        # 1. Try to resolve a pending future
        with self._pending_lock:
            for pending in self._pending[:]:
                if frame.code == pending.response_code and not pending.expired:
                    pending.future.set_result(frame)
                    self._pending.remove(pending)
                    break
            # Prune expired
            self._pending = [p for p in self._pending if not p.expired]

        # 2. Update local state
        self._update_state(frame)

        # 3. Fire telemetry callbacks
        for cb in self._telemetry_callbacks.get(frame.code, []):
            cb(frame, self._motor_states[frame.can_id])

    def _update_state(self, frame: Frame):
        code = frame.code
        d = frame.data
        if code == 0x31 and len(d) >= 7:
            raw = int.from_bytes(d[1:7], byteorder='big', signed=True)
            self._motor_states[frame.can_id].encoder_position_deg = counts_to_degrees(raw, self._counts_per_revolution)
        elif code == 0x32 and len(d) >= 3:
            self._motor_states[frame.can_id].speed_rpm = struct.unpack('>h', d[1:3])[0]
        elif code == 0x3A and len(d) >= 2:
            self._motor_states[frame.can_id].enabled = (d[1] == 0x01)
        elif code == 0x91 and len(d) >= 2:
            if d[1] == 2:   # homing complete
                print("%02X: homing complete" % frame.can_id)
                self._motor_states[frame.can_id].homed = True
                self._motor_states[frame.can_id].end_limit_hit = True
        elif code == 0xF5 and len(d) >= 2:
            if d[1] == 3:
                self._motor_states[frame.can_id].end_limit_hit = True
        elif code == 0xF3 and len(d) >= 2:
            enabled = (d[1] == 0x01)
            self._motor_states[frame.can_id].enabled = enabled

    # ── Telemetry subscriptions ───────────────────────────────────────────────
    def on_telemetry(self, code: int, callback: Callable[["Frame", MotorState], None]):
        """Register a callback for a specific response code.
        callback(frame, motor_state) is called from the callback thread.

        Useful codes:
          0x31 = position, 0x32 = speed, 0x39 = angle error
        """
        self._telemetry_callbacks.setdefault(code, []).append(callback)

    def on_enabled(self, callback):
        self.on_telemetry(0xF3, lambda f, s: callback(s.enabled))

    def on_homed(self, callback: Callable[[bool | None], None]):
        self.on_telemetry(0x91, lambda f, s: callback(True if f.data[1] == 2 else False))

    def on_position(self, callback: Callable[[float], None]):
        """Convenience: callback(position_deg)."""
        # if using a linear system, convert to linear position
        self.on_telemetry(
            0x31,
            lambda f, s: callback(
                 position_degrees_to_linear_position(s.encoder_position_deg, self._mm_per_revolution)
                 if self._mm_per_revolution else s.encoder_position_deg)
        )

    def on_speed(self, callback: Callable[[float], None]):
        """Convenience: callback(speed_rpm)."""
        self.on_telemetry(
            0x32,
            lambda f, s: callback(
                rpm_to_linear_velocity(s.speed_rpm, self._mm_per_revolution)
                if self._mm_per_revolution else float(s.speed_rpm))
        )

    def _build_data(self, payload: list[int], arbitration_id: int | None = None) -> list[int]:
        if arbitration_id is not None:
            return build(arbitration_id, payload)
        else:
            return build(self._can_id, payload)

    def _can_send(self, data: list, arbitration_id: int | None = None):
        if arbitration_id is None:
            self._bus.send(self._can_id, data)
        else:
            self._bus.send(arbitration_id, data)

    def _can_build_and_send_data(self, payload: list[int], arbitration_id: int | None = None):
        self._can_send(self._build_data(payload, arbitration_id), arbitration_id)

    def _can_send_with_reply(self, payload: list, response_code: int,
                             timeout: float = 0.5, arbitration_id: int | None = None) -> Future:
        pending = _PendingReply(response_code, timeout)
        with self._pending_lock:
            self._pending.append(pending)
        self._can_build_and_send_data(payload, arbitration_id)
        return pending.future

    def _can_send_with_reply_future(self, payload: list[int], response_code: int, timeout: float,
                                    arbitration_id: int | None = None) -> bytes | bytearray | None:
        reply = future_helper(self._can_send_with_reply(
            payload=payload,
            response_code=response_code,
            timeout=timeout,
            arbitration_id=arbitration_id
        ), timeout)
        return get_message_data(reply)

    def _clamp_inputs(self, degrees: float, speed_rpm: int, acceleration: int) -> tuple[float, int, int]:
        if self._max_position is not None:
            if degrees > self._max_position:
                degrees = self._max_position
                print("Desired move larger than max position, moving to max position: %.2f degrees" % float(degrees))
        if self._min_position is not None:
            if degrees < self._min_position:
                degrees = self._min_position
                print("Desired move lower than min position, moving to min position: %.2f degrees" % float(degrees))

        if speed_rpm > self._max_rpm:
            speed_rpm = self._max_rpm
            print("Desired angular velocity higher that maximum allowed angular velocity, clamped angular velocity to: %d RPM" % speed_rpm)

        if acceleration > self._max_mks_acceleration:
            acceleration = self._max_mks_acceleration
            print("Acceleration beyond limit was desired, clamped acceleration to: %d" % acceleration)
        elif acceleration < 0:
            acceleration = 1

        return degrees, speed_rpm, acceleration

    def check_move_allowed(self) -> bool:
        if all(state.enabled for state in self._motor_states.values()):
            if self._require_homing:
                if not all(state.homed for state in self._motor_states.values()):
                    #print("%02X: requires homing, no command sent" % self._can_id)
                    return False
            return True
        else:
            #print("%02X: not enabled warning, no command sent" % self._can_id)
            return False

    def check_requires_homing(self) -> bool:
        return self._require_homing

    def check_homed(self) -> bool:
        if self.check_requires_homing():
            return all(state.homed for state in self._motor_states.values())
        else:
            return True

    def get_can_id(self):
        return self._can_id

    def get_last_motor_enabled_state(self):
        return all(state.enabled for state in self._motor_states.values())

    # high = 1, CCW = 1, end_limit = 1, wire in_2 = low, home_speed = 120
    def cmd_set_home_configuration(self,
                                   home_trig: int,  # 0=low, 1=high triggered
                                   home_dir: int,  # 0=CW, 1=CCW
                                   home_speed: int,  # RPM
                                   end_limit: int,  # 0=disable, 1=enable
                                   hm_mode: int  # 0=switch, 1=mechanical, 2=single-turn
                                   ):
        """90H – Set homing and limit parameters. DLC=8"""
        spd_hi = (home_speed >> 8) & 0xFF
        spd_lo = home_speed & 0xFF
        payload = [0x90, home_trig, home_dir, spd_hi, spd_lo, end_limit, hm_mode]
        self._can_build_and_send_data(payload)

    def cmd_subscribe(self, code: int, interval_ms: int, arbitration_id: int | None = None) -> bool:
        """Ask the motor to push a parameter at the given interval.
        interval_ms=0 cancels. Returns Future resolved when ack received.
        """
        t_hi = (interval_ms >> 8) & 0xFF
        t_lo =  interval_ms       & 0xFF
        data = self._can_send_with_reply_future(
            payload=[0x01, code, t_hi, t_lo],
            response_code=0x01,
            timeout=0.2,
            arbitration_id=arbitration_id)
        command_success = False
        if data is not None:
            command_success = bool(data[2])
        msg_var = (arbitration_id if arbitration_id is not None else -1, "succeeded" if command_success else "failed")
        print("%02X: subscribe command %s" % msg_var)
        return command_success

    def cmd_read_parameter(self, code: int, timeout: float,
                           arbitration_id: int | None = None) -> bytes | bytearray | None:
        """0x00 – Read any system parameter by its command code. DLC=3
        e.g. pass 0x82 to read the current work mode.
        """
        payload = [0x00, code]
        return self._can_send_with_reply_future(
            payload=payload,
            response_code=code,
            timeout=timeout,
            arbitration_id=arbitration_id
        )

    def cmd_configure_response(self, x: int, y: int, arbitration_id: int | None = None):
        reply =  self._can_send_with_reply_future(
            payload=[0x8C, x, y],
            response_code=0x8C,
            timeout=0.2,
            arbitration_id=arbitration_id
        )
        success = False
        if reply is not None:
            if len(reply) > 2:
                if reply[1] == 0x01:
                    success = True
        print("%02X: configure response %s" % (arbitration_id if arbitration_id else self._can_id,
                                               "succeeded" if success else "failed"))
        return success

    def cmd_start_homing(self) -> bool:
        """91H – Execute homing. DLC=3
        goZeroMode=0x00 → physical return to zero (moves to switch).
        """
        payload = [0x91, 0x00]
        success = False
        reply = self._can_send_with_reply_future(payload, 0x91, 0.2, self._can_id)
        if reply is not None:
            if len(reply) > 2:
                if reply[1] > 0x00:
                    success = True
                else:
                    success = False
        print("%02X: %s initiating homing" % (self._can_id, "successfully" if success else "failed"))
        return success

    def cmd_set_group_ids(self):
        """8DH – Assigns motor nodes to a group CAN ID. DLC=4
        """
        id_hi = (self._can_id >> 8) & 0xFF
        id_lo = self._can_id & 0xFF
        payload = [0x8D, id_hi, id_lo]

        for node_id in self._node_ids:
            self._bus.send(node_id, build(node_id, payload))

    def cmd_check_foc_work_mode(self, arbitration_id: int) -> bool | None:
        result = self.cmd_read_parameter(0x82, 0.2, arbitration_id)
        end_result = None
        if result is not None:
            if len(result) > 2:
                if result[1] == 0x05:
                    end_result = True
                else:
                    end_result = False
        print("%02X: SR_vFOC work mode - %s" % (arbitration_id, end_result))
        return end_result

    def cmd_set_work_mode(self, mode: int = 0x05):
        """0x82 – Set SR_vFOC bus mode. Await before enabling."""
        return self._can_send_with_reply([0x82, mode], response_code=0x82)

    def _cmd_check_motor_enabled(self, arbitration_id: int) -> bool | None:
        self._can_send_with_reply_future(
            payload=[0x3A],
            response_code=0x3A,
            timeout=0.2,
            arbitration_id=arbitration_id,
        )
        # frame responds with a check
        return self._motor_states[arbitration_id].enabled

    def cmd_check_motors_enabled(self) -> bool:
        for motor_state in self._motor_states:
            response = self._cmd_check_motor_enabled(motor_state)
            print("%02X: checked if motor enabled, returned - %s" % (motor_state, response))
        return  all(state.enabled for state in self._motor_states.values())

    def _cmd_enable_motor(self, arbitration_id:int, enable: bool) -> bool:
        reply = self._can_send_with_reply_future(
            payload=[0xF3, 0x01 if enable else 0x00],
            response_code=0xF3,
            timeout=0.2,
            arbitration_id=arbitration_id
        )
        success: bool = False
        if reply is not None:
            if len(reply) > 2:
                if reply[1] == 0x01:
                    success = True
                    self._motor_states[arbitration_id].enabled = enable
                    if self._require_homing and not enable:
                        self._motor_states[arbitration_id].homed = False
                else:
                    success = False
        print("%02X: enable motor command %s, set enable status - %s"
              % (arbitration_id, "succeeded" if success else "failed", self._motor_states[arbitration_id].enabled))
        return success

    def cmd_enable_motors(self, enable: bool) -> bool:
        for node_key in self._motor_states:
            self._cmd_enable_motor(node_key, enable)

        return any(state.enabled for state in self._motor_states.values())

    def cmd_relative_angular_move(self, degrees: float, speed_rpm: int = 300, acceleration: int = 150):
        """F4H – relative move by given degrees."""
        if self.check_move_allowed():
            degrees, speed_rpm, acceleration = self._clamp_inputs(degrees, speed_rpm, acceleration)
            self._relative_command = True
            speed_bytes = rpm_to_speed_bytes(speed_rpm)
            pos_bytes = degrees_to_position_bytes(degrees, self._counts_per_revolution)
            payload = [0xF4, speed_bytes[0], speed_bytes[1], acceleration, pos_bytes[0], pos_bytes[1], pos_bytes[2]]
            self._can_build_and_send_data(payload)

    def cmd_set_zero(self):
        """0x92 – Set current position as zero/origin. DLC=3"""
        if not self._require_homing:
            payload = [0x92]
            self._can_build_and_send_data(payload)
        else:
            print("%02X: cannot zero as this motor uses an end stop for zero")

    def cmd_soft_stop(self, acceleration: float | int | None = None):
        if acceleration is not None:
            if self._mm_per_revolution is not None:
                acceleration = linear_acceleration_to_byte(acceleration, self._mm_per_revolution)
            else:
                acceleration = angular_acceleration_to_byte(acceleration)
            if acceleration > self._max_mks_acceleration:
                acceleration = self._max_mks_acceleration
            elif acceleration <= 0:
                acceleration = 1
        else:
            acceleration = self._max_mks_acceleration

        if self._relative_command:
            payload = [0xF4, 0x00, 0x00, acceleration, 0x00, 0x00, 0x00]
        else:
            positions = []
            for nodes in self._motor_states:
                positions.append(self._motor_states[nodes].encoder_position_deg)
            position_degrees = sum(positions) / len(positions)
            ax = degrees_to_position_bytes(position_degrees, self._counts_per_revolution)
            payload = [0xF5, 0x00, 0x00, acceleration, ax[0], ax[1], ax[2]]
        self._can_build_and_send_data(payload)

    def cmd_emergency_stop(self):
        payload = [0xF7]
        self._can_build_and_send_data(payload)

    def cmd_set_absolute_position_degrees(self, degrees: float,  speed_rpm: int = 300,
                                          acceleration_byte: int = 150) -> list[float | int] | None:
        """F5H – Move to absolute coordinate. DLC=8
        Position is in encoder counts relative to the zero point.
        Counts per revolution = 16384, so 90° = 4096 counts.
        """
        if self.check_move_allowed():
            degrees, speed_rpm, acceleration_byte = self._clamp_inputs(degrees, speed_rpm, acceleration_byte)
            self._relative_command = False
            s_hi = (speed_rpm >> 8) & 0x0F
            s_lo = speed_rpm & 0xFF
            ax = degrees_to_position_bytes(degrees, self._counts_per_revolution)
            payload = [0xF5, s_hi, s_lo, acceleration_byte, ax[0], ax[1], ax[2]]
            self._can_build_and_send_data(payload)
            # resend packet if end stop was hit (a bug with the MKS SERVO57D)
            if any(state.end_limit_hit for state in self._motor_states.values()):
                self._can_build_and_send_data(payload)
                for node_key in self._motor_states:
                    self._motor_states[node_key].end_limit_hit = False
            return [degrees, speed_rpm, acceleration_byte]
        return None

    def cmd_set_absolute_linear_position(self,
                                         distance_mm: float | int,
                                         speed_mmps: float = 20,
                                         acceleration_mmps2: float = 12.6) -> list[float | int] | None:
        if self._mm_per_revolution is not None:
            degrees = linear_position_to_position_degrees(distance_mm, self._mm_per_revolution)
            speed_rpm = linear_velocity_to_rpm(speed_mmps, self._mm_per_revolution)
            if acceleration_mmps2 > 0:
                acceleration_byte = linear_acceleration_to_byte(acceleration_mmps2, self._mm_per_revolution)
            else:
                acceleration_byte = 0
            sent_values = self.cmd_set_absolute_position_degrees(degrees, speed_rpm, acceleration_byte)
            if sent_values is not None:
                return [position_degrees_to_linear_position(sent_values[0], float(self._mm_per_revolution)),
                        rpm_to_linear_velocity(sent_values[1], float(self._mm_per_revolution)),
                        mks_acceleration_to_linear_acceleration(int(sent_values[2]), float(self._mm_per_revolution))]
        else:
            print("No action as linear moves are not setup for this device, use cmd_set_absolute_position_degrees instead")
        return None