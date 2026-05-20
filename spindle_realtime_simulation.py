from __future__ import annotations

import argparse
import curses
from dataclasses import dataclass, replace
from pathlib import Path

from spindle_plant_model import SpindlePlantModel, SpindlePlantParams
from spindle_simulation import (
    CoupledSimulationSession,
    RealTimeSimulationScheduler,
    SimulationCommand,
    SimulationSample,
)
from spindle_supervisor_model import SpindleSupervisorModel, SupervisorParams


STATE_NAMES = {
    0: "off",
    1: "bg_engage",
    2: "open_loop",
    3: "closed_loop",
    4: "fault",
}
SPARK_CHARS = " .:-=+*#%@"
DEFAULT_INI_PATH = Path(__file__).resolve().with_name("schaublin_125-CNC_2026-05-17-2.ini")
DEFAULT_HISTORY_LIMIT = 5000


@dataclass(frozen=True)
class TunableParameter:
    scope: str
    attr: str
    label: str
    step: float
    minimum: float | None = None
    maximum: float | None = None


TUNABLE_PARAMETERS = (
    TunableParameter("supervisor", "kp", "KP", 0.1, minimum=0.0),
    TunableParameter("supervisor", "ki", "KI", 0.1, minimum=0.0),
    TunableParameter("supervisor", "startup_hz", "STARTUP_HZ", 1.0, minimum=0.0),
    TunableParameter("supervisor", "cvt_target_hz", "CVT_TARGET_HZ", 1.0, minimum=0.0),
    TunableParameter("plant", "motor_time_constant", "MOTOR_TAU", 0.01, minimum=0.0),
    TunableParameter("plant", "encoder_noise_rpm", "ENCODER_NOISE", 1.0, minimum=0.0),
)


@dataclass
class OperatorPanelState:
    cmd_rpm: float = 0.0
    enable: bool = False
    brake: bool = False
    spindle_reverse: bool = False
    cvt_invert: bool = False
    gear_init: bool = False
    _backgear_pulse: bool = False
    _cvt_increase_pulse: bool = False
    _cvt_decrease_pulse: bool = False

    def clamp(self, max_spindle_rpm: float) -> None:
        self.cmd_rpm = max(0.0, min(self.cmd_rpm, max_spindle_rpm))

    def pulse_backgear(self) -> None:
        self._backgear_pulse = True

    def pulse_cvt_increase(self) -> None:
        self._cvt_increase_pulse = True

    def pulse_cvt_decrease(self) -> None:
        self._cvt_decrease_pulse = True

    def next_command(self) -> SimulationCommand:
        command = SimulationCommand(
            cmd_rpm=self.cmd_rpm,
            enable=self.enable,
            brake=self.brake,
            spindle_reverse=self.spindle_reverse,
            panel_backgear_btn=self._backgear_pulse,
            panel_cvt_increase=self._cvt_increase_pulse,
            panel_cvt_decrease=self._cvt_decrease_pulse,
            cvt_invert=self.cvt_invert,
            gear_init=self.gear_init,
        )
        self._backgear_pulse = False
        self._cvt_increase_pulse = False
        self._cvt_decrease_pulse = False
        return command


class InteractiveSimulationApp:
    def __init__(
        self,
        *,
        ini_path: Path,
        dt: float,
        ui_hz: float,
        history_limit: int,
        max_steps_per_update: int,
    ):
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if ui_hz <= 0.0:
            raise ValueError("ui_hz must be positive")
        if history_limit <= 0:
            raise ValueError("history_limit must be positive")
        if max_steps_per_update <= 0:
            raise ValueError("max_steps_per_update must be positive")

        self.ini_path = ini_path
        self.dt = dt
        self.ui_hz = ui_hz
        self.history_limit = history_limit
        self.max_steps_per_update = max_steps_per_update
        self.initial_supervisor_params = SupervisorParams.from_ini(ini_path)
        self.initial_plant_params = SpindlePlantParams()
        self.controls = OperatorPanelState()
        self.selected_param_index = 0
        self.paused = False
        self.quit_requested = False
        self._reset_session(preserve_params=False)

    def run(self, screen) -> None:
        try:
            curses.curs_set(0)
        except curses.error:
            pass

        screen.keypad(True)
        screen.timeout(0)
        frame_delay_ms = max(1, int(1000.0 / self.ui_hz))

        while not self.quit_requested:
            self._drain_input(screen)
            if not self.paused:
                self.scheduler.advance()
            self._render(screen)
            curses.napms(frame_delay_ms)

    def _drain_input(self, screen) -> None:
        while True:
            key = screen.getch()
            if key == -1:
                return
            self._handle_key(key)

    def _handle_key(self, key: int) -> None:
        if key in {ord("q"), ord("Q")}:
            self.quit_requested = True
            return

        if key == ord(" "):
            self.controls.enable = not self.controls.enable
            return

        if key in {ord("p"), ord("P")}:
            self.paused = not self.paused
            if not self.paused:
                self.scheduler.start()
            return

        if key in {ord("n"), ord("N")} and self.paused:
            self.session.step_command(self.controls.next_command())
            return

        if key == ord("0"):
            self._reset_session(preserve_params=True)
            return

        if key == ord("!"):
            self._reset_session(preserve_params=False)
            return

        if key in {ord("j"), curses.KEY_DOWN}:
            self._adjust_command_rpm(-10.0)
            return

        if key in {ord("k"), curses.KEY_UP}:
            self._adjust_command_rpm(10.0)
            return

        if key == ord("J"):
            self._adjust_command_rpm(-100.0)
            return

        if key == ord("K"):
            self._adjust_command_rpm(100.0)
            return

        if key in {ord("b"), ord("B")}:
            self.controls.pulse_backgear()
            return

        if key in {ord("i"), ord("I")}:
            self.controls.pulse_cvt_increase()
            return

        if key in {ord("d"), ord("D")}:
            self.controls.pulse_cvt_decrease()
            return

        if key in {ord("r"), ord("R")}:
            self.controls.spindle_reverse = not self.controls.spindle_reverse
            return

        if key in {ord("x"), ord("X")}:
            self.controls.brake = not self.controls.brake
            return

        if key in {ord("v"), ord("V")}:
            self.controls.cvt_invert = not self.controls.cvt_invert
            return

        if key == 9:
            self.selected_param_index = (self.selected_param_index + 1) % len(TUNABLE_PARAMETERS)
            return

        if key in {ord("+"), ord("=")}:
            self._adjust_selected_parameter(1.0)
            return

        if key == ord("-"):
            self._adjust_selected_parameter(-1.0)

    def _adjust_command_rpm(self, delta: float) -> None:
        self.controls.cmd_rpm += delta
        self.controls.clamp(self.session.controller.params.max_spindle_rpm)

    def _adjust_selected_parameter(self, direction: float) -> None:
        spec = TUNABLE_PARAMETERS[self.selected_param_index]
        if spec.scope == "supervisor":
            current_params = self.session.controller.params
            current_value = getattr(current_params, spec.attr)
            new_value = current_value + spec.step * direction
            new_value = self._clamp_value(new_value, spec)
            self.session.replace_supervisor_params(replace(current_params, **{spec.attr: new_value}))
            self.controls.clamp(self.session.controller.params.max_spindle_rpm)
            return

        current_params = self.session.plant.params
        current_value = getattr(current_params, spec.attr)
        new_value = current_value + spec.step * direction
        new_value = self._clamp_value(new_value, spec)
        self.session.replace_plant_params(replace(current_params, **{spec.attr: new_value}))

    def _clamp_value(self, value: float, spec: TunableParameter) -> float:
        if spec.minimum is not None:
            value = max(spec.minimum, value)
        if spec.maximum is not None:
            value = min(spec.maximum, value)
        return value

    def _reset_session(self, *, preserve_params: bool) -> None:
        if preserve_params and hasattr(self, "session"):
            supervisor_params = self.session.controller.params
            plant_params = self.session.plant.params
        else:
            supervisor_params = self.initial_supervisor_params
            plant_params = self.initial_plant_params

        self.controls = OperatorPanelState()
        self.controls.clamp(supervisor_params.max_spindle_rpm)
        controller = SpindleSupervisorModel(supervisor_params)
        plant = SpindlePlantModel(plant_params)
        self.session = CoupledSimulationSession(
            controller,
            plant,
            dt=self.dt,
            history_limit=self.history_limit,
        )
        self.scheduler = RealTimeSimulationScheduler(
            self.session,
            self.controls.next_command,
            max_steps_per_update=self.max_steps_per_update,
        )
        self.scheduler.start()
        self.paused = False

    def _render(self, screen) -> None:
        screen.erase()
        height, width = screen.getmaxyx()
        sample = self.session.last_sample
        control = self.session.last_control
        plant = self.session.last_plant_outputs
        params = self.session.controller.params
        plant_params = self.session.plant.params
        state_value = self.session.controller.state.state
        state_name = STATE_NAMES.get(state_value, str(state_value))
        dropped = self.scheduler.last_dropped_steps
        total_dropped = self.scheduler.total_dropped_steps

        self._draw_line(
            screen,
            0,
            (
                "Real-Time CVT Spindle Simulation "
                f"sim={self.session.time_s:8.3f}s steps={self.session.step_index:7d} "
                f"paused={'yes' if self.paused else 'no '} "
                f"dropped={dropped:3d}/{total_dropped:5d}"
            ),
        )
        self._draw_line(
            screen,
            1,
            "keys: q quit | space enable | p pause | n step | j/k +/-10 rpm | J/K +/-100 rpm | b backgear | i/d cvt +/-",
        )
        self._draw_line(
            screen,
            2,
            "keys: r reverse | x brake | v cvt invert | tab next param | +/- adjust param | 0 reset keep params | ! reset defaults",
        )

        if sample is None or control is None or plant is None:
            self._draw_line(screen, 4, "No samples yet. Controls will apply on the next simulation tick.")
            screen.refresh()
            return

        self._draw_line(
            screen,
            4,
            (
                f"cmd={sample.command_rpm:8.1f} rpm fb={sample.measured_fb_rpm:8.1f} rpm "
                f"spindle={sample.plant_spindle_rpm:8.1f} rpm motor_cmd={sample.motor_cmd_rpm:8.1f} rpm"
            ),
        )
        self._draw_line(
            screen,
            5,
            (
                f"motor_hz={sample.motor_hz_out:6.1f} ratio={sample.ratio_out:7.3f} fb_vel={sample.fb_vel:8.1f} "
                f"state={state_name:>11s} closed_loop={'yes' if sample.closed_loop else 'no '} fault={'yes' if sample.fault else 'no '}"
            ),
        )
        self._draw_line(
            screen,
            6,
            (
                f"enable={'on ' if self.controls.enable else 'off'} brake={'on ' if self.controls.brake else 'off'} "
                f"reverse={'on ' if self.controls.spindle_reverse else 'off'} cvt_invert={'on ' if self.controls.cvt_invert else 'off'} "
                f"backgear={'on ' if sample.backgear_state else 'off'}"
            ),
        )

        self._draw_line(screen, 8, "selected tunables (+/- adjust, tab cycles):")
        for offset, spec in enumerate(TUNABLE_PARAMETERS, start=0):
            line_index = 9 + offset
            if line_index >= height:
                break
            value = self._parameter_value(spec)
            marker = ">" if offset == self.selected_param_index else " "
            self._draw_line(
                screen,
                line_index,
                f"{marker} {spec.label:<14s} {value:10.3f} step={spec.step:6.3f}",
            )

        graph_top = 10 + len(TUNABLE_PARAMETERS)
        if graph_top < height:
            samples = self.session.history
            graph_width = max(8, width - 24)
            self._draw_line(
                screen,
                graph_top,
                self._graph_line(
                    "cmd",
                    graph_width,
                    samples,
                    lambda item: item.command_rpm,
                    max_value=params.max_spindle_rpm,
                ),
            )
        if graph_top + 1 < height:
            self._draw_line(
                screen,
                graph_top + 1,
                self._graph_line(
                    "fb",
                    graph_width,
                    samples,
                    lambda item: item.measured_fb_rpm,
                    max_value=params.max_spindle_rpm,
                ),
            )
        if graph_top + 2 < height:
            self._draw_line(
                screen,
                graph_top + 2,
                self._graph_line(
                    "motor_hz",
                    graph_width,
                    samples,
                    lambda item: item.motor_hz_out,
                    max_value=params.vfd_max_hz,
                ),
            )
        if graph_top + 3 < height:
            self._draw_line(
                screen,
                graph_top + 3,
                self._graph_line(
                    "ratio",
                    graph_width,
                    samples,
                    lambda item: item.ratio_out,
                    min_value=params.ratio_min,
                    max_value=params.ratio_max,
                ),
            )
        if graph_top + 4 < height:
            self._draw_line(
                screen,
                graph_top + 4,
                f"plant: tau={plant_params.motor_time_constant:.3f} encoder_noise={plant_params.encoder_noise_rpm:.1f} fb_plant={plant.fb_rpm:.1f}",
            )

        screen.refresh()

    def _parameter_value(self, spec: TunableParameter) -> float:
        if spec.scope == "supervisor":
            return float(getattr(self.session.controller.params, spec.attr))
        return float(getattr(self.session.plant.params, spec.attr))

    def _graph_line(
        self,
        label: str,
        width: int,
        samples: tuple[SimulationSample, ...],
        getter,
        *,
        min_value: float | None = 0.0,
        max_value: float | None = None,
    ) -> str:
        values = [getter(sample) for sample in samples]
        latest = values[-1] if values else 0.0
        spark = sparkline(values, width, min_value=min_value, max_value=max_value)
        return f"{label:<8s}{spark} {latest:9.3f}"

    def _draw_line(self, screen, row: int, text: str) -> None:
        height, width = screen.getmaxyx()
        if row < 0 or row >= height or width <= 1:
            return
        try:
            screen.addnstr(row, 0, text.ljust(width - 1), width - 1)
        except curses.error:
            return


def sparkline(
    values: list[float],
    width: int,
    *,
    min_value: float | None = None,
    max_value: float | None = None,
) -> str:
    if width <= 0:
        return ""
    if not values:
        return " " * width

    if len(values) > width:
        step = len(values) / width
        sampled = [values[int(index * step)] for index in range(width)]
    else:
        sampled = values

    lower = min_value if min_value is not None else min(sampled)
    upper = max_value if max_value is not None else max(sampled)
    if upper <= lower:
        graph = SPARK_CHARS[len(SPARK_CHARS) // 2] * len(sampled)
        return graph.rjust(width)

    char_count = len(SPARK_CHARS) - 1
    chars = []
    for value in sampled:
        normalized = (value - lower) / (upper - lower)
        normalized = max(0.0, min(1.0, normalized))
        index = int(round(normalized * char_count))
        chars.append(SPARK_CHARS[index])
    return "".join(chars).rjust(width)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the interactive real-time spindle simulation.")
    parser.add_argument(
        "--ini",
        type=Path,
        default=DEFAULT_INI_PATH,
        help="Path to the LinuxCNC INI file containing the [SPINDLE_0] section.",
    )
    parser.add_argument("--dt", type=float, default=0.001, help="Simulation step in seconds.")
    parser.add_argument("--ui-hz", type=float, default=20.0, help="Terminal redraw rate in Hz.")
    parser.add_argument(
        "--history",
        type=int,
        default=DEFAULT_HISTORY_LIMIT,
        help="Maximum number of recent simulation samples kept for the rolling trace view.",
    )
    parser.add_argument(
        "--max-steps-per-update",
        type=int,
        default=200,
        help="Maximum simulation steps allowed per UI frame before excess steps are dropped.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = InteractiveSimulationApp(
        ini_path=args.ini,
        dt=args.dt,
        ui_hz=args.ui_hz,
        history_limit=args.history,
        max_steps_per_update=args.max_steps_per_update,
    )
    curses.wrapper(app.run)


if __name__ == "__main__":
    main()