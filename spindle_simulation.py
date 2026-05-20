from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from spindle_plant_model import PlantOutputs, SpindlePlantModel
from spindle_supervisor_model import SpindleSupervisorModel, SupervisorInputs


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


def run_coupled_simulation(
    controller: SpindleSupervisorModel,
    plant: SpindlePlantModel,
    *,
    steps: int,
    dt: float,
    input_builder: InputBuilder,
    feedback_override: FeedbackOverride | None = None,
) -> list[SimulationSample]:
    samples: list[SimulationSample] = []

    for step_index in range(steps):
        time_s = step_index * dt
        effective_ratio = plant.state.cvt_ratio_high_gear
        if controller.state.bg_state:
            effective_ratio *= plant.params.backgear_multiplier
        plant_snapshot = PlantOutputs(
            motor_rpm=plant.state.motor_rpm,
            spindle_rpm=plant.state.spindle_rpm,
            fb_rpm=plant.state.spindle_rpm + plant.params.encoder_noise_rpm,
            effective_ratio=effective_ratio,
            cvt_ratio_high_gear=plant.state.cvt_ratio_high_gear,
            backgear_engaged=controller.state.bg_state,
        )
        measured_fb_rpm = plant_snapshot.fb_rpm
        if feedback_override is not None:
            override = feedback_override(step_index, time_s, plant_snapshot)
            if override is not None:
                measured_fb_rpm = override

        inputs = input_builder(step_index, time_s, measured_fb_rpm)
        control = controller.step(inputs, dt)
        plant_outputs = plant.step(control, dt)
        samples.append(
            SimulationSample(
                step_index=step_index,
                time_s=time_s,
                command_rpm=inputs.cmd_rpm,
                measured_fb_rpm=measured_fb_rpm,
                plant_fb_rpm=plant_outputs.fb_rpm,
                plant_spindle_rpm=plant_outputs.spindle_rpm,
                effective_ratio=plant_outputs.effective_ratio,
                ratio_out=control.ratio_out,
                fb_vel=controller.state.fb_vel,
                motor_cmd_rpm=control.motor_cmd_rpm,
                motor_hz_out=control.motor_hz_out,
                dbg_ratio_freeze=control.dbg_ratio_freeze,
                dbg_cvt_wait=control.dbg_cvt_wait,
                closed_loop=control.closed_loop,
                fault=control.fault,
                backgear_state=control.backgear_state,
            )
        )

    return samples