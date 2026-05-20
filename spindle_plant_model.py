from __future__ import annotations

from dataclasses import dataclass

from spindle_supervisor_model import SupervisorOutputs


@dataclass(frozen=True)
class SpindlePlantParams:
    motor_time_constant: float = 0.15
    high_gear_ratio_min: float = 0.493
    high_gear_ratio_max: float = 2.706
    backgear_multiplier: float = 6.5
    cvt_ratio_rate_per_sec: float = 0.35
    cvt_direction_inverted: bool = False
    cvt_stuck: bool = False
    encoder_noise_rpm: float = 0.0


@dataclass
class SpindlePlantState:
    motor_rpm: float = 0.0
    spindle_rpm: float = 0.0
    cvt_ratio_high_gear: float = 1.3


@dataclass(frozen=True)
class PlantOutputs:
    motor_rpm: float
    spindle_rpm: float
    fb_rpm: float
    effective_ratio: float
    cvt_ratio_high_gear: float
    backgear_engaged: bool


class SpindlePlantModel:
    def __init__(self, params: SpindlePlantParams, state: SpindlePlantState | None = None):
        self.params = params
        self.state = state or SpindlePlantState()

    def step(self, control: SupervisorOutputs, dt: float) -> PlantOutputs:
        state = self.state
        params = self.params

        if control.cvt_increase and not params.cvt_stuck:
            delta = -params.cvt_ratio_rate_per_sec * dt
            if params.cvt_direction_inverted:
                delta = -delta
            state.cvt_ratio_high_gear += delta
        elif control.cvt_decrease and not params.cvt_stuck:
            delta = params.cvt_ratio_rate_per_sec * dt
            if params.cvt_direction_inverted:
                delta = -delta
            state.cvt_ratio_high_gear += delta

        state.cvt_ratio_high_gear = max(
            params.high_gear_ratio_min,
            min(params.high_gear_ratio_max, state.cvt_ratio_high_gear),
        )

        motor_alpha = 1.0 if params.motor_time_constant <= 0.0 else min(1.0, dt / params.motor_time_constant)
        state.motor_rpm += (control.motor_cmd_rpm - state.motor_rpm) * motor_alpha

        effective_ratio = state.cvt_ratio_high_gear
        if control.backgear_state:
            effective_ratio *= params.backgear_multiplier

        state.spindle_rpm = state.motor_rpm / effective_ratio if effective_ratio > 0.0 else 0.0
        fb_rpm = state.spindle_rpm + params.encoder_noise_rpm

        return PlantOutputs(
            motor_rpm=state.motor_rpm,
            spindle_rpm=state.spindle_rpm,
            fb_rpm=fb_rpm,
            effective_ratio=effective_ratio,
            cvt_ratio_high_gear=state.cvt_ratio_high_gear,
            backgear_engaged=control.backgear_state,
        )