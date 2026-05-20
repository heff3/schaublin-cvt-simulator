from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time
from typing import Callable

from spindle_plant_model import PlantOutputs, SpindlePlantModel
from spindle_supervisor_model import SpindleSupervisorModel, SupervisorInputs, SupervisorOutputs


@dataclass(frozen=True)
class SimulationSample:
    step_index: int
    time_s: float
    command_rpm: float
    measured_fb_rpm: float
    plant_fb_rpm: float
    plant_spindle_rpm: float
    effective_ratio: float
    ratio_out: float
    fb_vel: float
    motor_cmd_rpm: float
    motor_hz_out: float
    dbg_ratio_freeze: float
    dbg_cvt_wait: float
    closed_loop: bool
    fault: bool
    backgear_state: bool


InputBuilder = Callable[[int, float, float], SupervisorInputs]
FeedbackOverride = Callable[[int, float, PlantOutputs], float | None]
TimeSource = Callable[[], float]


@dataclass(frozen=True)
class SimulationCommand:
    cmd_rpm: float = 0.0
    enable: bool = False
    brake: bool = False
    spindle_reverse: bool = False
    panel_backgear_btn: bool = False
    panel_cvt_increase: bool = False
    panel_cvt_decrease: bool = False
    cvt_invert: bool = False
    gear_init: bool = False

    def to_supervisor_inputs(self, fb_rpm: float) -> SupervisorInputs:
        return SupervisorInputs(
            cmd_rpm=self.cmd_rpm,
            fb_rpm=fb_rpm,
            enable=self.enable,
            brake=self.brake,
            spindle_reverse=self.spindle_reverse,
            panel_backgear_btn=self.panel_backgear_btn,
            panel_cvt_increase=self.panel_cvt_increase,
            panel_cvt_decrease=self.panel_cvt_decrease,
            cvt_invert=self.cvt_invert,
            gear_init=self.gear_init,
        )

    @classmethod
    def from_supervisor_inputs(cls, inputs: SupervisorInputs) -> "SimulationCommand":
        return cls(
            cmd_rpm=inputs.cmd_rpm,
            enable=inputs.enable,
            brake=inputs.brake,
            spindle_reverse=inputs.spindle_reverse,
            panel_backgear_btn=inputs.panel_backgear_btn,
            panel_cvt_increase=inputs.panel_cvt_increase,
            panel_cvt_decrease=inputs.panel_cvt_decrease,
            cvt_invert=inputs.cvt_invert,
            gear_init=inputs.gear_init,
        )


class CoupledSimulationSession:
    def __init__(
        self,
        controller: SpindleSupervisorModel,
        plant: SpindlePlantModel,
        *,
        dt: float,
        history_limit: int | None = None,
    ):
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if history_limit is not None and history_limit <= 0:
            raise ValueError("history_limit must be positive when provided")

        self.controller = controller
        self.plant = plant
        self.dt = dt
        self.step_index = 0
        self._history: deque[SimulationSample] = deque(maxlen=history_limit)
        self.last_inputs: SupervisorInputs | None = None
        self.last_control: SupervisorOutputs | None = None
        self.last_plant_snapshot: PlantOutputs | None = None
        self.last_plant_outputs: PlantOutputs | None = None
        self.last_sample: SimulationSample | None = None

    @property
    def time_s(self) -> float:
        return self.step_index * self.dt

    @property
    def history(self) -> tuple[SimulationSample, ...]:
        return tuple(self._history)

    def current_plant_snapshot(self) -> PlantOutputs:
        effective_ratio = self.plant.state.cvt_ratio_high_gear
        if self.controller.state.bg_state:
            effective_ratio *= self.plant.params.backgear_multiplier

        return PlantOutputs(
            motor_rpm=self.plant.state.motor_rpm,
            spindle_rpm=self.plant.state.spindle_rpm,
            fb_rpm=self.plant.state.spindle_rpm + self.plant.params.encoder_noise_rpm,
            effective_ratio=effective_ratio,
            cvt_ratio_high_gear=self.plant.state.cvt_ratio_high_gear,
            backgear_engaged=self.controller.state.bg_state,
        )

    def step_inputs(
        self,
        inputs: SupervisorInputs,
        *,
        measured_fb_rpm: float | None = None,
        plant_snapshot: PlantOutputs | None = None,
    ) -> SimulationSample:
        if plant_snapshot is None:
            plant_snapshot = self.current_plant_snapshot()
        if measured_fb_rpm is None:
            measured_fb_rpm = inputs.fb_rpm
        return self._step(inputs, plant_snapshot=plant_snapshot, measured_fb_rpm=measured_fb_rpm)

    def step_command(
        self,
        command: SimulationCommand,
        *,
        measured_fb_rpm: float | None = None,
        plant_snapshot: PlantOutputs | None = None,
    ) -> SimulationSample:
        if plant_snapshot is None:
            plant_snapshot = self.current_plant_snapshot()
        if measured_fb_rpm is None:
            measured_fb_rpm = plant_snapshot.fb_rpm
        inputs = command.to_supervisor_inputs(measured_fb_rpm)
        return self._step(inputs, plant_snapshot=plant_snapshot, measured_fb_rpm=measured_fb_rpm)

    def replace_supervisor_params(self, params) -> None:
        self.controller.params = params

    def replace_plant_params(self, params) -> None:
        self.plant.params = params

    def _step(
        self,
        inputs: SupervisorInputs,
        *,
        plant_snapshot: PlantOutputs,
        measured_fb_rpm: float,
    ) -> SimulationSample:
        step_index = self.step_index
        time_s = self.time_s
        control = self.controller.step(inputs, self.dt)
        plant_outputs = self.plant.step(control, self.dt)
        sample = SimulationSample(
            step_index=step_index,
            time_s=time_s,
            command_rpm=inputs.cmd_rpm,
            measured_fb_rpm=measured_fb_rpm,
            plant_fb_rpm=plant_outputs.fb_rpm,
            plant_spindle_rpm=plant_outputs.spindle_rpm,
            effective_ratio=plant_outputs.effective_ratio,
            ratio_out=control.ratio_out,
            fb_vel=self.controller.state.fb_vel,
            motor_cmd_rpm=control.motor_cmd_rpm,
            motor_hz_out=control.motor_hz_out,
            dbg_ratio_freeze=control.dbg_ratio_freeze,
            dbg_cvt_wait=control.dbg_cvt_wait,
            closed_loop=control.closed_loop,
            fault=control.fault,
            backgear_state=control.backgear_state,
        )
        self.step_index += 1
        self.last_inputs = inputs
        self.last_control = control
        self.last_plant_snapshot = plant_snapshot
        self.last_plant_outputs = plant_outputs
        self.last_sample = sample
        self._history.append(sample)
        return sample


@dataclass(frozen=True)
class SchedulerAdvance:
    samples: tuple[SimulationSample, ...]
    dropped_steps: int


class RealTimeSimulationScheduler:
    def __init__(
        self,
        session: CoupledSimulationSession,
        command_provider: Callable[[], SimulationCommand],
        *,
        time_source: TimeSource | None = None,
        max_steps_per_update: int = 20,
    ):
        if max_steps_per_update <= 0:
            raise ValueError("max_steps_per_update must be positive")

        self.session = session
        self.command_provider = command_provider
        self.time_source = time_source or time.monotonic
        self.max_steps_per_update = max_steps_per_update
        self.total_dropped_steps = 0
        self.last_dropped_steps = 0
        self._accumulator = 0.0
        self._last_time: float | None = None

    def start(self) -> None:
        self._accumulator = 0.0
        self.last_dropped_steps = 0
        self._last_time = self.time_source()

    def advance(self) -> SchedulerAdvance:
        now = self.time_source()
        if self._last_time is None:
            self._last_time = now
            return SchedulerAdvance(samples=(), dropped_steps=0)

        elapsed = max(0.0, now - self._last_time)
        self._last_time = now
        self._accumulator += elapsed

        due_steps = int(self._accumulator / self.session.dt)
        if due_steps <= 0:
            self.last_dropped_steps = 0
            return SchedulerAdvance(samples=(), dropped_steps=0)

        dropped_steps = max(0, due_steps - self.max_steps_per_update)
        steps_to_run = min(due_steps, self.max_steps_per_update)
        self._accumulator -= due_steps * self.session.dt
        self.last_dropped_steps = dropped_steps
        self.total_dropped_steps += dropped_steps

        samples: list[SimulationSample] = []
        for _ in range(steps_to_run):
            samples.append(self.session.step_command(self.command_provider()))
        return SchedulerAdvance(samples=tuple(samples), dropped_steps=dropped_steps)


def run_coupled_simulation(
    controller: SpindleSupervisorModel,
    plant: SpindlePlantModel,
    *,
    steps: int,
    dt: float,
    input_builder: InputBuilder,
    feedback_override: FeedbackOverride | None = None,
) -> list[SimulationSample]:
    session = CoupledSimulationSession(controller, plant, dt=dt, history_limit=steps)
    samples: list[SimulationSample] = []

    for _ in range(steps):
        step_index = session.step_index
        time_s = session.time_s
        plant_snapshot = session.current_plant_snapshot()
        measured_fb_rpm = plant_snapshot.fb_rpm
        if feedback_override is not None:
            override = feedback_override(step_index, time_s, plant_snapshot)
            if override is not None:
                measured_fb_rpm = override

        inputs = input_builder(step_index, time_s, measured_fb_rpm)
        samples.append(
            session.step_inputs(
                inputs,
                measured_fb_rpm=measured_fb_rpm,
                plant_snapshot=plant_snapshot,
            )
        )

    return samples