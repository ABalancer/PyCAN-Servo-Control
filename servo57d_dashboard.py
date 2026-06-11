"""
MKS SERVO57D – DearPyGui Control Dashboard
============================================
Requirements:
    pip install dearpygui python-can

CANable setup (slcan firmware):
    sudo slcand -o -s5 -t hw -S 3000000 /dev/ttyACM0 slcan0
    sudo ip link set slcan0 up

CANable setup (gs_usb / candleLight firmware):
    sudo ip link set can0 up type can bitrate 500000
    → change BUS_TYPE = "socketcan", INTERFACE = "can0"
"""

import threading
import time
import struct
import collections
import dearpygui.dearpygui as dpg
import can

# ── Configuration ─────────────────────────────────────────────────────────────
USB_PORT     = "COM10"
BUS_TYPE     = "slcan"    # or "socketcan"
CAN_BITRATE  = 500000
CAN_ID_LIST  = [0x0C, 0x0D, 0x0E, 0x0F]
CAN_ID_LIST_STR = [str(x) for x in CAN_ID_LIST]

COUNTS_PER_REV  = 16384
PLOT_HISTORY    = 200   # number of data points shown on the plots
POLL_INTERVAL   = 0.1   # seconds between telemetry polls

# ── Protocol helpers ──────────────────────────────────────────────────────────
def crc(can_id: int, payload: list[int]) -> int:
    return (can_id + sum(payload)) & 0xFF

def build(can_id: int, payload: list[int]) -> list:
    return payload + [crc(can_id, payload)]

def counts_to_degrees(counts: int) -> float:
    return counts / COUNTS_PER_REV * 360.0

def counts_to_ax(counts) -> bytes:
    ax = struct.pack('>i', counts)[1:]  # signed 24-bit big-endian
    return ax

def degrees_to_counts(deg: float) -> int:
    return round(deg / 360.0 * COUNTS_PER_REV)

def degrees_to_ax(deg: float) -> bytes:
    counts = degrees_to_counts(deg)
    ax = counts_to_ax(counts)
    return ax


class MotorInterface:
    def __init__(self):
        self._enabled = False
        self._lock = threading.Lock()
        self._pos_counts = 0
        self._speed_rpm  = 0
        self._can_id = CAN_ID_LIST[0]
        self._relative_command = False

        try:
            self._bus = can.Bus(interface=BUS_TYPE, channel="COM10", bitrate=CAN_BITRATE)
            # Start listener thread
            self._listener = threading.Thread(target=self._listen_loop, daemon=True)
            self._listener.start()
        except Exception as e:
            print(f"CAN bus open failed ({e}).")
            exit("Exiting Program.")
    # ── Internal ──────────────────────────────────────────────────────────────
    def _send(self, data: list):
        msg = can.Message(arbitration_id=self._can_id, data=data, is_extended_id=False)
        try:
            self._bus.send(msg)
        except Exception as e:
            print(f"CAN send error: {e}")

    def _listen_loop(self):
        """Background thread: parse incoming frames and update telemetry."""
        while True:
            try:
                msg = self._bus.recv(timeout=1.0)
                if msg is None or msg.arbitration_id != self._can_id:
                    continue
                d = msg.data
                if len(d) < 2:
                    continue
                code = d[0]
                if code == 0x31 and len(d) >= 8:
                    # Cumulative encoder: int48 big-endian in bytes 1-6
                    raw = int.from_bytes(d[1:7], byteorder='big', signed=True)
                    with self._lock:
                        self._pos_counts = raw
                elif code == 0x32 and len(d) >= 4:
                    # Speed: int16 big-endian in bytes 1-2
                    rpm = struct.unpack('>h', bytes(d[1:3]))[0]
                    with self._lock:
                        self._speed_rpm = rpm
            except Exception:
                pass

    # ── Public commands ───────────────────────────────────────────────────────
    def set_can_id(self, can_id: int):
        self._can_id = can_id

    def set_work_mode(self, mode: int = 0x05):
        """0x82 – SR_vFOC bus FOC mode (required before enable)."""
        self._send(build(self._can_id, [0x82, mode]))
        time.sleep(0.15)

    def get_can_id(self):
        return self._can_id

    def get_enable(self):
        return self._enabled

    def set_enable(self, enable: bool):
        self._send(build(self._can_id, [0xF3, 0x01 if enable else 0x00]))
        self._enabled = enable

    def jog(self, degrees: float, speed_rpm: int = 300, accel: int = 100):
        """F4H – relative move by given degrees."""
        self._relative_command = True
        s_hi  = (speed_rpm >> 8) & 0x0F
        s_lo  =  speed_rpm       & 0xFF
        ax = degrees_to_ax(degrees)
        data = build(self._can_id, [0xF4, s_hi, s_lo, accel, ax[0], ax[1], ax[2]])
        self._send(data)

    def set_zero(self):
        """0x92 – Set current position as zero/origin. DLC=3"""
        payload = [0x92]
        self._send(build(self._can_id, payload))

    def soft_stop(self, acceleration = 0xE6):
        current_position_counts = self._pos_counts
        if self._relative_command:
            payload = [0xF4, 0x00, 0x00, acceleration, 0x00, 0x00, 0x00]
        else:
            ax = counts_to_ax(current_position_counts)
            payload = [0xF5, 0x00, 0x00, acceleration, ax[0], ax[1], ax[2]]
        self._send(build(self._can_id, payload))

    def emergency_stop(self):
        payload = [0xF7]
        self._send(build(self._can_id, payload))

    def absolute_move(self, degrees: float, speed_rpm: int = 300, accel: int = 100):
        """F5H – Move to absolute coordinate. DLC=8
        Position is in encoder counts relative to the zero point.
        Counts per revolution = 16384, so 90° = 4096 counts.
        """
        self._relative_command = False
        s_hi = (speed_rpm >> 8) & 0x0F
        s_lo = speed_rpm & 0xFF
        ax = degrees_to_ax(degrees)
        payload = [0xF5, s_hi, s_lo, accel, ax[0], ax[1], ax[2]]
        self._send(build(self._can_id, payload))

    def poll_telemetry(self):
        """Send poll requests for position (31H) and speed (32H)."""
        self._send(build(self._can_id, [0x31]))
        self._send(build(self._can_id, [0x32]))

    def get_telemetry(self):
        """Return (position_degrees, speed_rpm)."""
        with self._lock:
            return counts_to_degrees(self._pos_counts), self._speed_rpm


# ── Dashboard ─────────────────────────────────────────────────────────────────
motor = MotorInterface()

# Rolling history buffers
_times    = collections.deque(maxlen=PLOT_HISTORY)
_pos_hist = collections.deque(maxlen=PLOT_HISTORY)
_spd_hist = collections.deque(maxlen=PLOT_HISTORY)
_t0 = time.time()


def _telemetry_loop():
    while True:
        motor.poll_telemetry()
        pos_deg, spd_rpm = motor.get_telemetry()
        t = time.time() - _t0
        _times.append(t)
        _pos_hist.append(pos_deg)
        _spd_hist.append(spd_rpm)

        ts  = list(_times)
        pos = list(_pos_hist)
        spd = list(_spd_hist)

        if dpg.is_dearpygui_running():
            try:
                dpg.set_value("pos_series", [ts, pos])
                dpg.set_value("spd_series", [ts, spd])
                dpg.fit_axis_data("pos_x_axis")
                dpg.fit_axis_data("spd_x_axis")
                dpg.set_value("pos_readout", f"{pos_deg:+.2f} °")
                dpg.set_value("spd_readout", f"{spd_rpm:+d} RPM")
            except Exception:
                pass

        time.sleep(POLL_INTERVAL)


def on_can_id_select(sender):
    new_can_id = int(dpg.get_value(sender))
    motor.set_can_id(new_can_id)


def on_enable_toggle(can_id: int):
    new_state = not motor.get_enable()
    previous_motor_id = motor.get_can_id()
    for motor_id in CAN_ID_LIST:
        motor.set_can_id(motor_id)
        if new_state:
            motor.set_work_mode(0x05)
        motor.set_enable(new_state)
    motor.set_can_id(previous_motor_id)
    if len(CAN_ID_LIST) > 1:
        label = "Disable Motors" if new_state else "Enable Motors"
    else:
        label = "Disable Motor" if new_state else "Enable Motor"
    #button_text_colour = (200, 60, 60, 255) if new_state else (60, 180, 60, 255)
    dpg.set_item_label("enable_btn", label)
    dpg.configure_item("enable_btn", **{"user_data": None})
    dpg.bind_item_theme("enable_btn", "btn_red" if new_state else "btn_green")
    dpg.set_value("status_text", "● ENABLED" if new_state else "○ DISABLED")
    dpg.configure_item("status_text", color=(80, 220, 80) if new_state else (180, 180, 180))


def on_jog(can_id: int, direction: float):
    dist = dpg.get_value("jog_distance")
    speed = dpg.get_value("jog_speed")
    accel = dpg.get_value("jog_accel")
    motor.jog(dist * direction, speed_rpm=speed, accel=accel)


def on_set_zero(can_id: int):
    motor.set_zero()


def on_soft_stop(can_id: int):
    accel = dpg.get_value("jog_accel")
    motor.soft_stop(accel)


def on_emergency_stop(can_id: int):
    motor.emergency_stop()


def on_absolute_move(can_id: int):
    movement = dpg.get_value("absolute_position")
    speed = dpg.get_value("jog_speed")
    accel = dpg.get_value("jog_accel")
    motor.absolute_move(movement, speed_rpm=speed, accel=accel)


def build_gui():
    app_width = 980
    app_height = 850

    dpg.create_context()
    dpg.create_viewport(title="SERVO57D CAN Dashboard", width=app_width, height=app_height,
                        resizable=True)

    # ── Themes ────────────────────────────────────────────────────────────────
    with dpg.font_registry():
        regular_font = dpg.add_font("./CascadiaCodePL.ttf", 16)

    with dpg.theme(tag="btn_green"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (45, 150, 70))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (60, 180, 90))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (30, 120, 55))

    with dpg.theme(tag="btn_red"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (170, 45, 45))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (210, 65, 65))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (140, 30, 30))

    with dpg.theme(tag="btn_move"):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (50, 100, 190))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (70, 130, 220))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (35, 75, 155))

    with dpg.theme(tag="global_theme"):
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding,  6)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,   5)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing,     8, 8)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding,    6, 5)
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg,        (22, 24, 30))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg,         (38, 42, 52))
            dpg.add_theme_color(dpg.mvThemeCol_Header,          (50, 90, 150))
            dpg.add_theme_color(dpg.mvThemeCol_Text,            (220, 225, 235))

    dpg.bind_theme("global_theme")

    # ── Main window ───────────────────────────────────────────────────────────
    with dpg.window(tag="main_win", label="", no_title_bar=True,
                    no_move=True, no_resize=True,
                    width=app_width, height=app_height, pos=(0, 0)):

        dpg.bind_font(regular_font)
        dpg.add_text("MKS SERVO57D — CAN Dashboard", color=(130, 180, 255))
        dpg.add_text(f"CAN Module Port: {USB_PORT} ({BUS_TYPE})", color=(120, 120, 140))
        dpg.add_separator()
        dpg.add_spacer(height=6)

        # ── Top row: enable + live readouts ───────────────────────────────────
        with dpg.group(horizontal=True):
            # Enable/Disable button
            with dpg.child_window(width=200, height=160, border=True, no_scrollbar=True):
                dpg.add_spacer(height=2)

                dpg.add_button(label="Enable Motors" if len(CAN_ID_LIST) > 1 else "Enable Motor", tag="enable_btn",
                               width=130, height=40,
                               callback=on_enable_toggle)
                dpg.bind_item_theme("enable_btn", "btn_green")

                dpg.add_spacer(height=2)

                dpg.add_text("○ DISABLED", tag="status_text",
                             color=(180, 180, 180))

            dpg.add_spacer(width=6)

            # Live readouts
            with dpg.child_window(width=200, height=160, border=True):
                with dpg.group(horizontal=True):
                    dpg.add_text("CAN ID:", tag="can_id_text",
                                 color=(180, 180, 180))

                    dpg.add_combo(items=CAN_ID_LIST_STR, default_value=CAN_ID_LIST_STR[0],
                                  callback=on_can_id_select,
                                  width=80)

                dpg.add_spacer(height=2)

                with dpg.group(horizontal=True):
                    dpg.add_text("Position:", color=(130, 180, 255))
                    dpg.add_text("+0.00 °", tag="pos_readout",
                                 color=(255, 220, 80))

                dpg.add_spacer(height=2)

                with dpg.group(horizontal=True):
                    dpg.add_text("Speed:", color=(130, 180, 255))
                    dpg.add_text("+0 RPM", tag="spd_readout",
                                 color=(100, 220, 160))

            dpg.add_spacer(width=6)

            # Jog controls
            with dpg.child_window(width=400, height=160, border=True):
                with dpg.group(horizontal=True):
                    with dpg.group():
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="↑↓", tag="jog_cw", #▶▶
                                           width=70, height=40,
                                           callback=lambda: on_jog(motor.get_can_id(), +1.0))
                            dpg.bind_item_theme("jog_cw", "btn_move")

                            dpg.add_button(label="↓↑", tag="jog_ccw", #◀◀
                                           width=70, height=40,
                                           callback=lambda: on_jog(motor.get_can_id(), -1.0))
                            dpg.bind_item_theme("jog_ccw", "btn_move")

                        with dpg.group(horizontal=True):
                            dpg.add_button(label="▶▶", tag="abs_move",  # ◀◀
                                           width=70, height=40,
                                           callback=lambda: on_absolute_move(motor.get_can_id()))
                            dpg.bind_item_theme("abs_move", "btn_move")

                            dpg.add_button(label="■", tag="soft_stop",  # ▶▶
                                           width=70, height=40,
                                           callback=lambda: on_soft_stop(motor.get_can_id()))
                            dpg.bind_item_theme("soft_stop", "btn_move")

                        with dpg.group(horizontal=True):
                            dpg.add_button(label="⌂", tag="home",
                                           width=70, height=40,
                                           callback=lambda: on_set_zero(motor.get_can_id()))
                            dpg.bind_item_theme("home", "btn_home")

                            dpg.add_button(label="■", tag="emergency_stop",
                                           width=70, height=40,
                                           callback=lambda: on_emergency_stop(motor.get_can_id()))
                            dpg.bind_item_theme("emergency_stop", "btn_red")

                    dpg.add_spacer(width=6)
                    with dpg.group():
                        with dpg.group(horizontal=True):
                            dpg.add_text("Pos  °", color=(160,160,180))
                            dpg.add_input_float(tag="absolute_position",
                                                default_value=0,
                                                min_value=-7200, max_value=7200,
                                                step=45, width=150)

                        with dpg.group(horizontal=True):
                            dpg.add_text("Dist °", color=(160,160,180))
                            dpg.add_input_float(tag="jog_distance",
                                                default_value=720,
                                                min_value=-7200, max_value=7200,
                                                step=45, width=150)
                        with dpg.group(horizontal=True):
                            dpg.add_text("Speed ", color=(160,160,180))
                            dpg.add_input_int(tag="jog_speed",
                                              default_value=600,
                                              min_value=1, max_value=3000,
                                              step=50, width=150)
                        with dpg.group(horizontal=True):
                            dpg.add_text("Accel ", color=(160,160,180))
                            dpg.add_input_int(tag="jog_accel",
                                              default_value=200,
                                              min_value=0, max_value=255,
                                              step=5, width=150)

                    dpg.add_spacer(width=6)

        dpg.add_spacer(height=10)

        # ── Plots ─────────────────────────────────────────────────────────────
        with dpg.group(horizontal=False):

            # Position plot
            with dpg.plot(label="Position (°)", height=240, width=-1,
                          anti_aliased=True):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label="time (s)",
                                  tag="pos_x_axis")
                with dpg.plot_axis(dpg.mvYAxis, label="degrees",
                                   tag="pos_y_axis"):
                    dpg.add_line_series([], [], label="Position °",
                                       tag="pos_series")
                    dpg.bind_item_theme(
                        "pos_series",
                        _make_series_theme((255, 220, 80)))

            dpg.set_axis_limits("pos_y_axis", -10000, 10000)
            dpg.add_spacer(height=6)

            # Speed plot
            with dpg.plot(label="Speed (RPM)", height=240, width=-1,
                          anti_aliased=True):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis, label="time (s)",
                                  tag="spd_x_axis")
                with dpg.plot_axis(dpg.mvYAxis, label="RPM",
                                   tag="spd_y_axis"):
                    dpg.add_line_series([], [], label="Speed RPM",
                                       tag="spd_series")
                    dpg.bind_item_theme(
                        "spd_series",
                        _make_series_theme((100, 220, 160)))

            dpg.set_axis_limits("spd_y_axis", -1000, 1000)

    dpg.setup_dearpygui()
    dpg.show_viewport()
    # Resize main window whenever viewport changes
    with dpg.item_handler_registry(tag="viewport_handler"):
        pass
    dpg.set_primary_window("main_win", True)


def _make_series_theme(color: tuple):
    """Helper: create a line-series colour theme and return its tag."""
    tag = f"series_theme_{color}"
    if not dpg.does_item_exist(tag):
        with dpg.theme(tag=tag):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, color,
                                    category=dpg.mvThemeCat_Plots)
    return tag


def main():
    build_gui()
    # Start telemetry polling thread
    t = threading.Thread(target=_telemetry_loop, daemon=True)
    t.start()
    dpg.start_dearpygui()
    if motor.get_enable():
        motor.set_enable(False)
        time.sleep(0.2)
    dpg.destroy_context()


if __name__ == "__main__":
    main()
