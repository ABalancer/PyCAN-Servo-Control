"""
DearPyGui Gantry Control Dashboard for MKS SERVO57D motors
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

import time
import collections
import dearpygui.dearpygui as dpg
import colorsys
from MotorSystem import MotorSystem
import json

'''todo
Test with multiple motors

unify node responses within one motor interface. Then report single responses representing all nodes.
They are likely currently bugs with regards to above.

disable movement replies when in use. Just track via telemetry and warn if trajectory error large / no movement.
must assign group IDs and use those for the above.
'''


# ── Configuration ─────────────────────────────────────────────────────────────
USB_PORT: str                      = "COM5"
BUS_TYPE: str                      = "slcan" # or "socketcan"
CAN_BITRATE: int                   = 500000  # default rate of SERVO57D
PLOT_HISTORY: int                  = 200     # number of data points shown on the plots
PLOT_UPDATE_FREQUENCY: float | int = 10.0      # FPS


class Stream:
    def __init__(self, name: str, history_length: int, start_time: float | None= None):
        self._start_time = start_time if start_time else time.monotonic()
        self._time: collections.deque[float] = collections.deque(maxlen=history_length)
        self._data: collections.deque[float] = collections.deque(maxlen=history_length)
        self._name: str = name
        self._latest_value: float = 0.0
        self._updated = False

    def append(self, value: float):
        self._time.append(time.monotonic() - self._start_time)
        self._data.append(value)
        self._latest_value = value
        self._updated = True

    def get_name(self) -> str:
        return self._name

    def get_data(self) -> tuple[list[float], list[float]]:
        self._updated = False
        return list(self._time), list(self._data)

    def new_data_available(self) -> bool:
        return self._updated

    def get_latest_value(self) -> float:
        return self._latest_value


# yellow: (255, 220, 80)
# green: (100, 220, 160)
def generate_distinct_colours(keys: list[str], saturation=0.6, value=1.0):
    n: int = len(keys)
    colours: dict[str, tuple[int, int, int]] = {}
    for i, key in enumerate(keys):
        h = i / n
        r, g, b = colorsys.hsv_to_rgb(h, saturation, value)
        colours[key] = (int(r * 255), int(g * 255), int(b * 255))
    return colours

# noinspection bad-context-manager
def make_series_theme(colour: tuple):
    """Helper: create a line-series colour theme and return its tag."""
    tag = f"series_theme_{colour}"
    if not dpg.does_item_exist(tag):
        with dpg.theme(tag=tag):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, colour,
                                    category=dpg.mvThemeCat_Plots)
    return tag

# noinspection bad-context-manager
class SystemGUI:
    def __init__(self, motor_system: MotorSystem):
        device_keys: list[str] = motor_system.get_system_keys()
        device_keys_length: int = len(device_keys)
        self._motor_system: MotorSystem = motor_system
        self._selected_motor_key: str = device_keys[-1]
        self._colours: dict[str, tuple[int, int, int]] = generate_distinct_colours(device_keys)
        self._dpg_axis_selector: dict[str, list] = {key: [] for key in device_keys}
        t0 = time.monotonic()
        self._streams: dict[str, Stream] = {key: Stream(key, PLOT_HISTORY, t0) for key in device_keys}
        self._combo_menu = False

        app_width: int = 980
        app_height: int = 794

        spacer_width: int = 6
        padding: int = 5
        rounding: int = 10

        dpg.create_context()
        dpg.create_viewport(title="CAN System Dashboard", width=app_width, height=app_height,
                            resizable=False)

        # ── Themes ────────────────────────────────────────────────────────────────
        with dpg.font_registry():
            regular_font = dpg.add_font("./CascadiaCodePL.ttf", 16)

        with dpg.theme(tag="btn_green"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (45, 150, 70))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (60, 180, 90))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (30, 120, 55))

        with dpg.theme(tag="btn_red"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (170, 45, 45))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (210, 65, 65))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (140, 30, 30))

        with dpg.theme(tag="btn_move"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (50, 100, 190))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (70, 130, 220))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (35, 75, 155))
            with dpg.theme_component(dpg.mvButton, enabled_state=False):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (51, 51, 51))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (51, 51, 51))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (51, 51, 51))

        with dpg.theme(tag="global_theme"):
            with dpg.theme_component(dpg.mvAll):

                dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, rounding)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, rounding)
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, rounding)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, padding, padding)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, padding, padding)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 0, 0)
                dpg.add_theme_style(dpg.mvStyleVar_ItemInnerSpacing, 5, 5)
                dpg.add_theme_style(dpg.mvStyleVar_CellPadding, 5, 5)
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (22, 24, 30))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (38, 42, 52))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (50, 90, 150))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (220, 225, 235))

        dpg.bind_theme("global_theme")

        # ── Main window ───────────────────────────────────────────────────────────
        with dpg.window(tag="main_win", label="", no_title_bar=True,
                        no_move=True, no_resize=True,
                        width=app_width, height=app_height, pos=(0, 0)):
            dpg.bind_font(regular_font)
            dpg.add_text("CAN Dashboard for SERVO57D System", color=(130, 180, 255))
            dpg.add_text(f"CAN Module Port: {USB_PORT} ({BUS_TYPE})", color=(120, 120, 140))
            dpg.add_separator()
            dpg.add_spacer(height=spacer_width)

            # ── Top row: enable + live readouts ───────────────────────────────────
            child_window_height = 160
            with dpg.group(horizontal=True):
                dpg.add_spacer(width=spacer_width)
                # Enable/Disable button
                with dpg.child_window(width=150, height=child_window_height, border=True, no_scrollbar=True):

                    dpg.add_button(label="Enable System", tag="enable_btn",
                                   width=150 - 2 * padding, height=40,
                                   callback=self._on_enable_system_toggle)
                    dpg.bind_item_theme("enable_btn", "btn_green")

                    dpg.add_spacer(height=2)

                    dpg.add_text("○ DISABLED", tag="status_text",
                                 color=(180, 180, 180))

                dpg.add_spacer(width=spacer_width)

                # Live readouts
                can_window_width = 250
                with dpg.child_window(width=can_window_width, height=child_window_height, border=True, no_scrollbar=True,
                                      no_scroll_with_mouse=True):
                    if device_keys_length > 5:
                        self._combo_menu = True
                        with dpg.group(horizontal=True):
                            dpg.add_text("Module:", tag="module_text",
                                         color=(180, 180, 180))

                            combo_tag = dpg.add_combo(items=device_keys, default_value=self._selected_motor_key,
                                                callback=self._on_can_combo_select, width=80)


                        dpg.add_spacer(height=2)

                        with dpg.group(horizontal=True):
                            dpg.add_text("Position:", color=(160, 160, 180))
                            position_text_tag = dpg.add_text("%7.2f" % 0.0, tag="position_readout",
                                                             color=(255, 220, 80))
                            dpg.add_text("mm", color=(160, 160, 180))
                        for key in self._dpg_axis_selector.keys():
                            dpg_list = [combo_tag, position_text_tag]
                            self._dpg_axis_selector[key] = dpg_list.copy()

                    else:
                        with dpg.group(horizontal=False):
                            dpg_item_list: list = [0, 0]
                            button_height = int((child_window_height - (device_keys_length + 1) * padding)
                                                / device_keys_length)
                            padding_height = button_height - 16 - 4 * padding
                            for key in device_keys:
                                with dpg.group(horizontal=True):
                                    dpg_item_list[0] = dpg.add_button(
                                        label=key, callback=self._on_can_button_select, user_data=key,
                                        width=button_height, height=button_height)
                                    with dpg.group(horizontal_spacing=0):
                                        dpg.add_spacer(height=padding_height)
                                        with dpg.group(horizontal=True):
                                            dpg.add_text(":", color=(160, 160, 180))
                                            dpg_item_list[1] = dpg.add_text("%7.4f" % 0.0, color=self._colours[key])
                                            dpg.add_text("mm", color=(160, 160, 180))
                                    self._dpg_axis_selector[key] = dpg_item_list.copy()
                                if key == self._selected_motor_key:
                                    dpg.bind_item_theme(self._dpg_axis_selector[key][0], "btn_move")
                                else:
                                    dpg.bind_item_theme(self._dpg_axis_selector[key][0], "global_theme")

                    dpg.add_spacer(height=2)
                dpg.add_spacer(width=spacer_width)

                # Jog controls
                button_height = int((child_window_height - padding * (3 + 1)) / 3)
                with dpg.child_window(width=500, height=child_window_height, border=True):
                    with dpg.group(horizontal=True):
                        with dpg.group():
                            with dpg.group(horizontal=True):
                                dpg.add_button(label="↓↑", tag="jog_cw",  # ▶▶
                                               width=70, height=button_height,
                                               callback=lambda: self._on_jog(+1.0))
                                dpg.bind_item_theme("jog_cw", "btn_move")

                                dpg.add_button(label="↑↓", tag="jog_ccw",  # ◀◀
                                               width=70, height=button_height,
                                               callback=lambda: self._on_jog(-1.0))
                                dpg.bind_item_theme("jog_ccw", "btn_move")

                            with dpg.group(horizontal=True):
                                dpg.add_button(label="▶▶", tag="abs_move",  # ◀◀
                                               width=70, height=button_height,
                                               callback=self._on_absolute_move)
                                dpg.bind_item_theme("abs_move", "btn_move")

                                dpg.add_button(label="■", tag="soft_stop",  # ▶▶
                                               width=70, height=button_height,
                                               callback=self._on_soft_stop)
                                dpg.bind_item_theme("soft_stop", "btn_move")

                            with dpg.group(horizontal=True):
                                dpg.add_button(label="⌂", tag="home",
                                               width=70, height=button_height,
                                               callback=self._on_go_home)
                                dpg.bind_item_theme("home", "btn_move")

                                dpg.add_button(label="■", tag="emergency_stop",
                                               width=70, height=button_height,
                                               callback=self._on_emergency_stop)
                                dpg.bind_item_theme("emergency_stop", "btn_red")

                        dpg.add_spacer(width=6)
                        spacer_height = 19 - (padding * 2)
                        with dpg.group():
                            dpg.add_spacer(height=spacer_height)
                            with dpg.group(horizontal=True):
                                dpg.add_text("Position  mm", color=(160, 160, 180))
                                dpg.add_input_float(tag="absolute_position",
                                                    default_value=0,
                                                    min_value=-7200, max_value=7200,
                                                    step=45, width=150)
                            dpg.add_spacer(height=spacer_height)
                            with dpg.group(horizontal=True):
                                dpg.add_text("Jog       mm", color=(160, 160, 180))
                                dpg.add_input_float(tag="jog_distance",
                                                    default_value=10.0, min_value=-1000.0, max_value=1000.0, step=100.0,
                                                    width=150)
                            dpg.add_spacer(height=spacer_height)
                            with dpg.group(horizontal=True):
                                dpg.add_text("Speed   mm/s", color=(160, 160, 180))
                                dpg.add_input_float(tag="jog_speed",
                                                    default_value=40.0, min_value=1.0, max_value=200.0, step=25.0,
                                                    width=150)
                            dpg.add_spacer(height=spacer_height)
                            with dpg.group(horizontal=True):
                                dpg.add_text("Accel mm/s^2", color=(160, 160, 180))
                                dpg.add_input_float(tag="jog_accel",
                                                    default_value=25.0, min_value=1.0, max_value=200.0, step=25.0,
                                                    width=150)

                        dpg.add_spacer(width=6)
            dpg.add_spacer(height=spacer_width)

            # ── Plots ─────────────────────────────────────────────────────────────
            with dpg.group(horizontal=False):
                # Position plot
                with dpg.group(horizontal=True):
                    dpg.add_spacer(width=spacer_width)
                    with dpg.plot(label="Motor Positions", height=500, width=app_width-12-36,
                                  anti_aliased=True):
                        dpg.add_plot_legend()
                        dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)",
                                          tag="pos_x_axis")
                        with dpg.plot_axis(dpg.mvYAxis, label="Position (mm)",
                                           tag="pos_y_axis"):
                            for key in device_keys:
                                line_series = dpg.add_line_series([], [], label=key + " Position",
                                                                  tag=key + "_plot_series")
                                dpg.bind_item_theme(key + "_plot_series", make_series_theme(self._colours[key]))
                                self._dpg_axis_selector[key].append(line_series)

                    dpg.set_axis_limits("pos_y_axis", -225, 5)
                    dpg.add_spacer(width=spacer_width)
                dpg.add_spacer(height=spacer_width)

        dpg.setup_dearpygui()
        dpg.show_viewport()
        # Resize main window whenever viewport changes
        with dpg.item_handler_registry(tag="viewport_handler"):
            pass
        dpg.set_primary_window("main_win", True)

    def _initialise_callbacks(self):
        for key in self._motor_system.get_system_keys():
            self._motor_system.set_active_axis(key)
            self._motor_system.get_active_axis_object().on_homed(lambda h: self._set_buttons_enabled_state(h))
            self._motor_system.get_active_axis_object().on_position(lambda pos:self._streams[key].append(pos))

        self._set_enable_button_colour(self._motor_system.get_system_enabled())
        self._set_buttons_enabled_state()

    def start_loop(self):
        self._initialise_callbacks()
        next_update: float | int = time.monotonic()

        while dpg.is_dearpygui_running():
            now: float | int = time.monotonic()

            if now >= next_update:
                self._update_plot()
                next_update += 1.0 / PLOT_UPDATE_FREQUENCY

                # Recover if the process was delayed for a long time.
                if next_update < now:
                    next_update = now

            dpg.render_dearpygui_frame()

        if self._motor_system.get_system_enabled():
            self._on_enable_system_toggle(None, None, None)
        self._motor_system.close_can()
        dpg.destroy_context()

    def _update_plot(self) -> bool:
        update: bool = False
        for key in self._streams.keys():
            if self._streams[key].new_data_available():
                dpg.set_value(self._dpg_axis_selector[key][2], self._streams[key].get_data())
                if not self._combo_menu:
                    dpg.set_value(self._dpg_axis_selector[key][1], "%7.4f" % self._streams[key].get_latest_value())
                else:
                    if key == self._motor_system.get_active_axis_name():
                        dpg.set_value(self._dpg_axis_selector[key][1], "%7.4f" % self._streams[key].get_latest_value())
                update = True
        if update:
            dpg.fit_axis_data("pos_x_axis")
        return update

    def _enable_move_buttons(self, state: bool):
        if not state and state is not None:
            dpg.disable_item("jog_cw")
            dpg.disable_item("jog_ccw")
            dpg.disable_item("abs_move")
        else:
            dpg.enable_item("jog_cw")
            dpg.enable_item("jog_ccw")
            dpg.enable_item("abs_move")

    def _enable_home_button(self, state: bool):
        if not state and state is not None:
            dpg.disable_item("home")
        else:
            dpg.enable_item("home")

    def _set_buttons_enabled_state(self, force_set_state: bool | None = None):
        if force_set_state is None:
            axis_move_allowed = self._motor_system.check_axis_move_allowed()
            motor_enabled = self._motor_system.check_axis_enabled()
            if axis_move_allowed:
                self._enable_move_buttons(True)
                self._enable_home_button(True)
            elif motor_enabled:
                self._enable_move_buttons(False)
                self._enable_home_button(True)
            else:
                self._enable_move_buttons(False)
                self._enable_home_button(False)
        else:
            self._enable_move_buttons(force_set_state)
            self._enable_home_button(force_set_state)

    def _set_enable_button_colour(self, state: bool):
        label = "Disable System" if state else "Enable System"
        # button_text_colour = (200, 60, 60, 255) if new_state else (60, 180, 60, 255)
        dpg.set_item_label("enable_btn", label)
        dpg.configure_item("enable_btn", **{"user_data": None})
        dpg.bind_item_theme("enable_btn", "btn_red" if state else "btn_green")
        dpg.set_value("status_text", "● ENABLED" if state else "○ DISABLED")
        dpg.configure_item("status_text", color=(80, 220, 80) if state else (180, 180, 180))

    def _on_can_button_select(self, sender, app_data, user_data):
        self._selected_motor_key = user_data  # label of the clicked button
        self._motor_system.set_active_axis(self._selected_motor_key)
        self._set_buttons_enabled_state()
        for button_key in self._dpg_axis_selector:
            if button_key == self._selected_motor_key:
                dpg.bind_item_theme(self._dpg_axis_selector[button_key][0], "btn_move")
            else:
                dpg.bind_item_theme(self._dpg_axis_selector[button_key][0], "global_theme")

    def _on_can_combo_select(self, sender, app_data, user_data):
        self._selected_motor_key = dpg.get_value(sender)
        self._motor_system.set_active_axis(self._selected_motor_key)
        self._set_buttons_enabled_state()

    def _on_enable_system_toggle(self, sender, app_data, user_data):
        enable_state = self._motor_system.get_system_enabled()
        new_state = not enable_state
        if enable_state:
            self._set_buttons_enabled_state(new_state)

        checked_state = self._motor_system.toggle_motor_enable()
        self._set_enable_button_colour(checked_state)
        self._set_buttons_enabled_state()

    def _on_absolute_move(self, sender, app_data, user_data):
        motor_key = self._selected_motor_key
        position = dpg.get_value("absolute_position")
        speed = dpg.get_value("jog_speed")
        acceleration = dpg.get_value("jog_accel")
        self._motor_system.set_axis_position(position, speed, acceleration, motor_key)

    def _on_jog(self, direction: float):
        dist = dpg.get_value("jog_distance")
        speed = dpg.get_value("jog_speed")
        acceleration = dpg.get_value("jog_accel")
        self._motor_system.jog_axis(dist * direction, speed, acceleration, self._selected_motor_key)

    def _on_go_home(self, sender, app_data, user_data):
        motor_key = self._selected_motor_key
        if self._motor_system.check_axis_requires_homing():
            self._set_buttons_enabled_state(False)
            self._motor_system.home_axis(motor_key)
        else:
            self._motor_system.zero_axis(motor_key)

    def _on_soft_stop(self, sender, app_data, user_data):
        motor_key = self._selected_motor_key
        acceleration = dpg.get_value("jog_accel")
        self._motor_system.stop_axis(acceleration, motor_key)
        if self._motor_system.get_system_enabled():
            self._enable_home_button(True)

    def _on_emergency_stop(self, sender, app_data, user_data):
        motor_key = self._selected_motor_key
        self._motor_system.stop_axis(0, motor_key)
        if self._motor_system.get_system_enabled():
            self._enable_home_button(True)


if __name__ == "__main__":
    with open("device_config.json", "r") as f:
        device_dictionary: dict[str, dict] = json.load(f)
    system = MotorSystem(device_dictionary, BUS_TYPE, USB_PORT, CAN_BITRATE)
    gui = SystemGUI(system)
    gui.start_loop()
