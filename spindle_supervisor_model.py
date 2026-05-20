from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SupervisorParams:
    vfd_min_hz: float
    vfd_max_hz: float
    motor_rpm_per_hz: float
    startup_hz: float
    ol_stable_time: float
    ol_max_time: float
    ol_stable_band: float
    max_spindle_rpm: float
    ratio_first: float
    ratio_alpha: float
    ratio_freeze_time: float
    ratio_gate_time: float
    ratio_min: float
    ratio_max: float
    kp: float
    ki: float
    deadband_rpm: float
    max_i_error: float
    cvt_target_hz: float
    cvt_hz_band: float
    cvt_hz_band_bg: float
    cvt_hz_per_sec: float
    cvt_pulse_min: float
    cvt_pulse_max: float
    cvt_settle_time: float
    backgear_threshold: float
    backgear_ratio_min: float
    bg_engage_time: float
    bg_spin_threshold: float
    cvt_invert: bool = False

    @classmethod
    def from_ini(cls, ini_path: str | Path) -> "SupervisorParams":
        section: dict[str, str] = {}
        current_section: str | None = None
        with Path(ini_path).open("r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.split("#", 1)[0].split(";", 1)[0].strip()
                if not line:
                    continue
                if line.startswith("[") and line.endswith("]"):
                    current_section = line[1:-1]
                    continue
                if current_section != "SPINDLE_0" or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                section[key.strip()] = value.strip()

        if not section:
            raise KeyError("[SPINDLE_0] section not found in INI file")

        def getfloat(name: str) -> float:
            return float(section[name])

        return cls(
            vfd_min_hz=getfloat("VFD_MIN_HZ"),
            vfd_max_hz=getfloat("VFD_MAX_HZ"),
            motor_rpm_per_hz=getfloat("MOTOR_RPM_PER_HZ"),
            startup_hz=getfloat("STARTUP_HZ"),
            ol_stable_time=getfloat("OL_STABLE_TIME"),
            ol_max_time=getfloat("OL_MAX_TIME"),
            ol_stable_band=getfloat("OL_STABLE_BAND"),
            max_spindle_rpm=getfloat("MAX_SPINDLE_RPM"),
            ratio_first=getfloat("RATIO_FIRST"),
            ratio_alpha=getfloat("RATIO_ALPHA"),
            ratio_freeze_time=getfloat("RATIO_FREEZE_TIME"),
            ratio_gate_time=getfloat("RATIO_GATE_TIME"),
            ratio_min=getfloat("RATIO_MIN"),
            ratio_max=getfloat("RATIO_MAX"),
            kp=getfloat("KP"),
            ki=getfloat("KI"),
            deadband_rpm=getfloat("DEADBAND_RPM"),
            max_i_error=getfloat("MAX_I_ERROR"),
            cvt_target_hz=getfloat("CVT_TARGET_HZ"),
            cvt_hz_band=getfloat("CVT_HZ_BAND"),
            cvt_hz_band_bg=getfloat("CVT_HZ_BAND_BG"),
            cvt_hz_per_sec=getfloat("CVT_HZ_PER_SEC"),
            cvt_pulse_min=getfloat("CVT_PULSE_MIN"),
            cvt_pulse_max=getfloat("CVT_PULSE_MAX"),
            cvt_settle_time=getfloat("CVT_SETTLE_TIME"),
            backgear_threshold=getfloat("BACKGEAR_THRESHOLD"),
            backgear_ratio_min=getfloat("BACKGEAR_RATIO_MIN"),
            bg_engage_time=getfloat("BG_ENGAGE_TIME"),
            bg_spin_threshold=getfloat("BG_SPIN_THRESHOLD"),
        )


@dataclass(frozen=True)
class SupervisorInputs:
    cmd_rpm: float = 0.0
    fb_rpm: float = 0.0
    enable: bool = False
    brake: bool = False
    spindle_reverse: bool = False
    panel_backgear_btn: bool = False
    panel_cvt_increase: bool = False
    panel_cvt_decrease: bool = False
    cvt_invert: bool = False
    gear_init: bool = False


@dataclass
class SupervisorState:
    state: int = 0
    state_timer: float = 0.0
    ol_timer: float = 0.0
    ol_stable_timer: float = 0.0
    prev_fb_abs: float = 0.0
    fb_vel: float = 0.0
    ratio_filt: float = 0.0
    ratio_init_done: bool = False
    gear_init_done: bool = False
    bg_state: bool = False
    want_backgear: bool = False
    integrator: float = 0.0
    cvt_pulse_timer: float = 0.0
    cvt_gate_timer: float = 0.0
    ratio_freeze_timer: float = 0.0
    cvt_dir: int = 0
    motor_cmd_rpm: float = 0.0
    panel_inc_count: int = 0
    panel_dec_count: int = 0
    last_bg_btn: bool = False
    last_enable: bool = False


@dataclass(frozen=True)
class SupervisorOutputs:
    motor_cmd_rpm: float
    vfd_brake: bool
    backgear_on: bool
    backgear_off: bool
    cvt_increase: bool
    cvt_decrease: bool
    backgear_state: bool
    fault: bool
    closed_loop: bool
    ratio_out: float
    motor_hz_out: float
    active_kp: float
    active_ki: float
    dbg_error_rpm: float
    dbg_integrator: float
    dbg_cvt_wait: float
    dbg_ol_stable: float
    dbg_state: float
    dbg_ratio_freeze: float
    dbg_motor_hz_err: float
    dbg_cvt_pulse_calc: float
    dbg_fb_rpm_abs: float


class SpindleSupervisorModel:
    def __init__(self, params: SupervisorParams, state: SupervisorState | None = None):
        self.params = params
        self.state = state or SupervisorState()

    def step(self, inputs: SupervisorInputs, dt: float) -> SupervisorOutputs:
        params = self.params
        state = self.state

        max_motor_rpm = params.vfd_max_hz * params.motor_rpm_per_hz
        min_motor_rpm = params.vfd_min_hz * params.motor_rpm_per_hz

        if not state.ratio_init_done:
            state.ratio_filt = params.ratio_first
            state.ratio_init_done = True

        if not state.gear_init_done:
            state.bg_state = bool(inputs.gear_init)
            state.gear_init_done = True

        fb_abs = abs(inputs.fb_rpm)
        cmd = max(0.0, min(inputs.cmd_rpm, params.max_spindle_rpm))

        raw = abs((fb_abs - state.prev_fb_abs) / dt)
        state.fb_vel = state.fb_vel * 0.99 + raw * 0.01
        state.prev_fb_abs = fb_abs

        state.ratio_freeze_timer = max(0.0, state.ratio_freeze_timer - dt)
        state.cvt_gate_timer = max(0.0, state.cvt_gate_timer - dt)

        backgear_on = bool(state.bg_state)
        backgear_off = not backgear_on
        vfd_brake = bool(inputs.brake)
        fault = False
        closed_loop = state.state == 3
        motor_cmd_rpm = 0.0
        motor_hz_out = 0.0
        active_kp = params.kp / (state.ratio_filt + 0.001)
        active_ki = 0.0
        dbg_error_rpm = 0.0
        dbg_integrator = state.integrator
        dbg_cvt_wait = state.cvt_gate_timer
        dbg_ol_stable = state.ol_stable_timer
        dbg_state = float(state.state)
        dbg_ratio_freeze = state.ratio_freeze_timer
        dbg_motor_hz_err = 0.0
        dbg_cvt_pulse_calc = 0.0
        cvt_increase = False
        cvt_decrease = False
        previous_motor_cmd_rpm = state.motor_cmd_rpm

        bg_edge = inputs.panel_backgear_btn and not state.last_bg_btn
        state.last_bg_btn = inputs.panel_backgear_btn
        if bg_edge and not inputs.enable and fb_abs < params.bg_spin_threshold and state.state in {0, 4}:
            state.bg_state = not state.bg_state
            backgear_on = bool(state.bg_state)
            backgear_off = not backgear_on

        enable_edge = inputs.enable and not state.last_enable
        state.last_enable = inputs.enable

        if state.cvt_pulse_timer > 0.0:
            state.cvt_pulse_timer -= dt
            cvt_increase, cvt_decrease = self._pulse_outputs(state.cvt_dir, inputs.cvt_invert or params.cvt_invert)
            if state.cvt_pulse_timer <= 0.0:
                state.cvt_pulse_timer = 0.0
                state.cvt_dir = 0
                cvt_increase = False
                cvt_decrease = False

        if state.state == 4:
            state.motor_cmd_rpm = 0.0
            fault = True
            closed_loop = False
            state.bg_state = False
            backgear_on = False
            backgear_off = True
            if not inputs.enable:
                state.state = 0
                fault = False
            return self._build_outputs(
                motor_cmd_rpm=motor_cmd_rpm,
                vfd_brake=vfd_brake,
                backgear_on=backgear_on,
                backgear_off=backgear_off,
                cvt_increase=cvt_increase,
                cvt_decrease=cvt_decrease,
                backgear_state=state.bg_state,
                fault=fault,
                closed_loop=closed_loop,
                ratio_out=state.ratio_filt,
                motor_hz_out=motor_hz_out,
                active_kp=active_kp,
                active_ki=active_ki,
                dbg_error_rpm=dbg_error_rpm,
                dbg_integrator=state.integrator,
                dbg_cvt_wait=state.cvt_gate_timer,
                dbg_ol_stable=state.ol_stable_timer,
                dbg_state=float(state.state),
                dbg_ratio_freeze=state.ratio_freeze_timer,
                dbg_motor_hz_err=dbg_motor_hz_err,
                dbg_cvt_pulse_calc=dbg_cvt_pulse_calc,
                dbg_fb_rpm_abs=fb_abs,
            )

        if state.state == 0:
            state.motor_cmd_rpm = 0.0
            state.integrator = 0.0
            state.cvt_pulse_timer = 0.0
            state.cvt_gate_timer = params.cvt_settle_time
            state.cvt_dir = 0
            state.ratio_freeze_timer = 0.0
            state.ol_timer = 0.0
            state.ol_stable_timer = 0.0
            dbg_integrator = 0.0
            dbg_cvt_wait = state.cvt_gate_timer
            active_ki = 0.0
            if enable_edge and cmd > 0.0:
                state.want_backgear = cmd <= params.backgear_threshold
                if state.want_backgear != state.bg_state:
                    state.bg_state = state.want_backgear
                    state.state = 1
                    state.state_timer = 0.0
                else:
                    state.state = 2
                    state.ol_timer = 0.0
                    state.ol_stable_timer = 0.0
            return self._build_outputs(
                motor_cmd_rpm=0.0,
                vfd_brake=vfd_brake,
                backgear_on=bool(state.bg_state),
                backgear_off=not bool(state.bg_state),
                cvt_increase=cvt_increase,
                cvt_decrease=cvt_decrease,
                backgear_state=state.bg_state,
                fault=False,
                closed_loop=False,
                ratio_out=state.ratio_filt,
                motor_hz_out=0.0,
                active_kp=params.kp / (state.ratio_filt + 0.001),
                active_ki=0.0,
                dbg_error_rpm=0.0,
                dbg_integrator=0.0,
                dbg_cvt_wait=state.cvt_gate_timer,
                dbg_ol_stable=0.0,
                dbg_state=float(state.state),
                dbg_ratio_freeze=state.ratio_freeze_timer,
                dbg_motor_hz_err=0.0,
                dbg_cvt_pulse_calc=0.0,
                dbg_fb_rpm_abs=fb_abs,
            )

        if state.state == 1:
            state.motor_cmd_rpm = 0.0
            backgear_on = bool(state.bg_state)
            backgear_off = not backgear_on
            if not inputs.enable:
                state.bg_state = not state.want_backgear
                return self._build_outputs(
                    motor_cmd_rpm=0.0,
                    vfd_brake=vfd_brake,
                    backgear_on=bool(state.bg_state),
                    backgear_off=not bool(state.bg_state),
                    cvt_increase=cvt_increase,
                    cvt_decrease=cvt_decrease,
                    backgear_state=state.bg_state,
                    fault=False,
                    closed_loop=False,
                    ratio_out=state.ratio_filt,
                    motor_hz_out=0.0,
                    active_kp=params.kp / (state.ratio_filt + 0.001),
                    active_ki=0.0,
                    dbg_error_rpm=0.0,
                    dbg_integrator=state.integrator,
                    dbg_cvt_wait=state.cvt_gate_timer,
                    dbg_ol_stable=0.0,
                    dbg_state=float(state.state),
                    dbg_ratio_freeze=state.ratio_freeze_timer,
                    dbg_motor_hz_err=0.0,
                    dbg_cvt_pulse_calc=0.0,
                    dbg_fb_rpm_abs=fb_abs,
                )
            state.state_timer += dt
            if state.state_timer >= params.bg_engage_time:
                state.state = 2
                state.ol_timer = 0.0
                state.ol_stable_timer = 0.0
            return self._build_outputs(
                motor_cmd_rpm=0.0,
                vfd_brake=vfd_brake,
                backgear_on=backgear_on,
                backgear_off=backgear_off,
                cvt_increase=cvt_increase,
                cvt_decrease=cvt_decrease,
                backgear_state=state.bg_state,
                fault=False,
                closed_loop=False,
                ratio_out=state.ratio_filt,
                motor_hz_out=0.0,
                active_kp=params.kp / (state.ratio_filt + 0.001),
                active_ki=0.0,
                dbg_error_rpm=0.0,
                dbg_integrator=state.integrator,
                dbg_cvt_wait=state.cvt_gate_timer,
                dbg_ol_stable=0.0,
                dbg_state=float(state.state),
                dbg_ratio_freeze=state.ratio_freeze_timer,
                dbg_motor_hz_err=0.0,
                dbg_cvt_pulse_calc=0.0,
                dbg_fb_rpm_abs=fb_abs,
            )

        if state.state == 2:
            state.ol_timer += dt
            ol_motor_rpm = params.startup_hz * params.motor_rpm_per_hz
            motor_cmd_rpm = ol_motor_rpm
            state.motor_cmd_rpm = motor_cmd_rpm
            motor_hz_out = params.startup_hz
            dbg_error_rpm = cmd - fb_abs
            backgear_on = bool(state.bg_state)
            backgear_off = not backgear_on

            if fb_abs > 2.0 and state.fb_vel < params.ol_stable_band:
                state.ol_stable_timer += dt
            else:
                state.ol_stable_timer = 0.0
            dbg_ol_stable = state.ol_stable_timer

            if fb_abs > 2.0 and fb_abs < (ol_motor_rpm / params.ratio_min) and fb_abs > (ol_motor_rpm / params.ratio_max):
                ratio_sample = ol_motor_rpm / (fb_abs + 0.001)
                if params.ratio_min < ratio_sample < params.ratio_max:
                    state.ratio_filt = state.ratio_filt * 0.80 + ratio_sample * 0.20

            active_kp = params.kp / (state.ratio_filt + 0.001)

            stable = state.ol_stable_timer >= params.ol_stable_time and fb_abs > 2.0
            timeout = state.ol_timer >= params.ol_max_time
            if not inputs.enable:
                state.state = 0
            elif stable or timeout:
                if state.want_backgear and state.ratio_filt < params.backgear_ratio_min:
                    state.bg_state = False
                    state.state = 4
                else:
                    if cmd > 0.001:
                        ratio_max_safe = max_motor_rpm / cmd
                        if state.ratio_filt > ratio_max_safe:
                            state.ratio_filt = ratio_max_safe
                    state.integrator = 0.0
                    state.cvt_gate_timer = params.cvt_settle_time
                    state.cvt_dir = 0
                    state.ratio_freeze_timer = 0.0
                    state.state = 3

            return self._build_outputs(
                motor_cmd_rpm=motor_cmd_rpm,
                vfd_brake=vfd_brake,
                backgear_on=backgear_on,
                backgear_off=backgear_off,
                cvt_increase=cvt_increase,
                cvt_decrease=cvt_decrease,
                backgear_state=state.bg_state,
                fault=state.state == 4,
                closed_loop=state.state == 3,
                ratio_out=state.ratio_filt,
                motor_hz_out=motor_hz_out,
                active_kp=active_kp,
                active_ki=0.0,
                dbg_error_rpm=dbg_error_rpm,
                dbg_integrator=state.integrator,
                dbg_cvt_wait=state.cvt_gate_timer,
                dbg_ol_stable=state.ol_stable_timer,
                dbg_state=float(state.state),
                dbg_ratio_freeze=state.ratio_freeze_timer,
                dbg_motor_hz_err=0.0,
                dbg_cvt_pulse_calc=0.0,
                dbg_fb_rpm_abs=fb_abs,
            )

        if state.state != 3:
            raise ValueError(f"Unexpected supervisor state: {state.state}")

        closed_loop = True
        if not inputs.enable:
            state.state = 0
            state.motor_cmd_rpm = 0.0
            closed_loop = False
            return self._build_outputs(
                motor_cmd_rpm=0.0,
                vfd_brake=vfd_brake,
                backgear_on=bool(state.bg_state),
                backgear_off=not bool(state.bg_state),
                cvt_increase=cvt_increase,
                cvt_decrease=cvt_decrease,
                backgear_state=state.bg_state,
                fault=False,
                closed_loop=closed_loop,
                ratio_out=state.ratio_filt,
                motor_hz_out=0.0,
                active_kp=params.kp / (state.ratio_filt + 0.001),
                active_ki=0.0,
                dbg_error_rpm=0.0,
                dbg_integrator=state.integrator,
                dbg_cvt_wait=state.cvt_gate_timer,
                dbg_ol_stable=state.ol_stable_timer,
                dbg_state=float(state.state),
                dbg_ratio_freeze=state.ratio_freeze_timer,
                dbg_motor_hz_err=0.0,
                dbg_cvt_pulse_calc=0.0,
                dbg_fb_rpm_abs=fb_abs,
            )

        if state.ratio_freeze_timer <= 0.0:
            can_learn = (
                fb_abs > 5.0
                and previous_motor_cmd_rpm < max_motor_rpm * 0.95
                and previous_motor_cmd_rpm > min_motor_rpm * 1.05
                and state.cvt_pulse_timer <= 0.0
            )
            if can_learn:
                ratio_sample = previous_motor_cmd_rpm / (fb_abs + 0.001)
                if params.ratio_min < ratio_sample < params.ratio_max:
                    state.ratio_filt = state.ratio_filt * (1.0 - params.ratio_alpha) + ratio_sample * params.ratio_alpha

        sched_kp = params.kp / (state.ratio_filt + 0.001)
        sched_ki = params.ki / (state.ratio_filt + 0.001)
        active_kp = sched_kp
        active_ki = sched_ki

        error = cmd - fb_abs
        dbg_error_rpm = error
        active_db = params.deadband_rpm * 0.5 if state.ratio_freeze_timer > 0.0 else params.deadband_rpm
        if error > active_db:
            e_db = error - active_db
        elif error < -active_db:
            e_db = error + active_db
        else:
            e_db = 0.0

        i_clamp = params.max_i_error * (state.ratio_filt + 0.001) / (params.ki + 0.001)
        state.integrator += e_db * dt
        state.integrator = max(-i_clamp, min(i_clamp, state.integrator))
        dbg_integrator = state.integrator

        correction = sched_kp * e_db + sched_ki * state.integrator
        demand = (cmd + correction) * state.ratio_filt
        if demand > max_motor_rpm:
            demand = max_motor_rpm
            if e_db > 0.0:
                state.integrator -= e_db * dt
        if 0.0 < demand < min_motor_rpm:
            demand = min_motor_rpm
        if demand < 0.0:
            demand = 0.0

        motor_cmd_rpm = demand
        state.motor_cmd_rpm = motor_cmd_rpm
        motor_hz_out = demand / params.motor_rpm_per_hz
        hz_err = motor_hz_out - params.cvt_target_hz
        dbg_motor_hz_err = hz_err
        active_band = params.cvt_hz_band_bg if state.bg_state else params.cvt_hz_band
        unclamped_hz = (cmd * state.ratio_filt) / (params.motor_rpm_per_hz + 0.001)
        unclamped_hz_err = unclamped_hz - params.cvt_target_hz

        if state.cvt_pulse_timer <= 0.0 and fb_abs > 10.0 and cmd > 0.0:
            want_inc = False
            want_dec = False
            state.panel_inc_count = state.panel_inc_count + 1 if inputs.panel_cvt_increase else 0
            state.panel_dec_count = state.panel_dec_count + 1 if inputs.panel_cvt_decrease else 0
            inc_edge = state.panel_inc_count == 5
            dec_edge = state.panel_dec_count == 5
            if inc_edge:
                want_inc = True
            elif dec_edge:
                want_dec = True
            elif state.cvt_gate_timer <= 0.0 and not inputs.spindle_reverse:
                at_max = motor_hz_out >= params.vfd_max_hz - 1.0
                at_min = motor_hz_out <= params.vfd_min_hz + 1.0
                short_rpm = cmd - fb_abs > params.deadband_rpm
                over_rpm = fb_abs - cmd > params.deadband_rpm
                if (hz_err > active_band and not at_max) or (at_max and short_rpm):
                    want_inc = True
                elif (hz_err < -active_band and not at_min and not short_rpm) or (at_min and over_rpm):
                    want_dec = True

            if want_inc or want_dec:
                calc = abs(unclamped_hz_err) / (params.cvt_hz_per_sec + 0.001)
                calc = max(params.cvt_pulse_min, min(params.cvt_pulse_max, calc))
                dbg_cvt_pulse_calc = calc
                state.cvt_dir = 1 if want_inc else -1
                state.cvt_pulse_timer = calc
                state.cvt_gate_timer = params.ratio_gate_time
                state.ratio_freeze_timer = params.ratio_freeze_time
                cvt_increase, cvt_decrease = self._pulse_outputs(state.cvt_dir, inputs.cvt_invert or params.cvt_invert)

        return self._build_outputs(
            motor_cmd_rpm=motor_cmd_rpm,
            vfd_brake=vfd_brake,
            backgear_on=bool(state.bg_state),
            backgear_off=not bool(state.bg_state),
            cvt_increase=cvt_increase,
            cvt_decrease=cvt_decrease,
            backgear_state=state.bg_state,
            fault=False,
            closed_loop=closed_loop,
            ratio_out=state.ratio_filt,
            motor_hz_out=motor_hz_out,
            active_kp=active_kp,
            active_ki=active_ki,
            dbg_error_rpm=dbg_error_rpm,
            dbg_integrator=dbg_integrator,
            dbg_cvt_wait=state.cvt_gate_timer,
            dbg_ol_stable=state.ol_stable_timer,
            dbg_state=float(state.state),
            dbg_ratio_freeze=state.ratio_freeze_timer,
            dbg_motor_hz_err=dbg_motor_hz_err,
            dbg_cvt_pulse_calc=dbg_cvt_pulse_calc,
            dbg_fb_rpm_abs=fb_abs,
        )

    @staticmethod
    def _pulse_outputs(cvt_dir: int, invert: bool) -> tuple[bool, bool]:
        if not invert:
            return cvt_dir == 1, cvt_dir == -1
        return cvt_dir == -1, cvt_dir == 1

    @staticmethod
    def _build_outputs(**kwargs: float | bool) -> SupervisorOutputs:
        return SupervisorOutputs(**kwargs)