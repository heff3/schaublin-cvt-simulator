from pathlib import Path

from spindle_plant_model import SpindlePlantModel, SpindlePlantParams, SpindlePlantState
from spindle_simulation import run_coupled_simulation
from spindle_supervisor_model import SpindleSupervisorModel, SupervisorInputs, SupervisorParams, SupervisorState


ROOT = Path(__file__).resolve().parents[1]
INI_PATH = ROOT / "schaublin_125-CNC_2026-05-17-2.ini"


def load_params() -> SupervisorParams:
    return SupervisorParams.from_ini(INI_PATH)


def test_backgear_encoder_drop_blocks_ratio_learning_under_high_fb_vel():
    params = load_params()
    controller = SpindleSupervisorModel(
        params,
        SupervisorState(
            state=3,
            ratio_filt=7.989,
            ratio_init_done=True,
            gear_init_done=True,
            bg_state=True,
            motor_cmd_rpm=1137.0,
            prev_fb_abs=202.0,
            cvt_gate_timer=0.0,
            ratio_freeze_timer=0.0,
        ),
    )
    plant = SpindlePlantModel(
        SpindlePlantParams(motor_time_constant=0.0),
        SpindlePlantState(
            motor_rpm=1137.0,
            spindle_rpm=1137.0 / 7.989,
            cvt_ratio_high_gear=7.989 / 6.5,
        ),
    )

    samples = run_coupled_simulation(
        controller,
        plant,
        steps=10,
        dt=0.001,
        input_builder=lambda _step_index, _time_s, measured_fb_rpm: SupervisorInputs(
            cmd_rpm=134.0,
            fb_rpm=measured_fb_rpm,
            enable=True,
        ),
        feedback_override=lambda _step_index, _time_s, _plant_snapshot: 37.0,
    )

    ratio_trace = [sample.ratio_out for sample in samples]
    ratio_steps = [after - before for before, after in zip(ratio_trace, ratio_trace[1:])]

    assert all(sample.fb_vel > params.ol_stable_band * 3.0 for sample in samples[:8])
    assert ratio_trace == [7.989] * len(ratio_trace)
    assert max(abs(step) for step in ratio_steps) == 0.0
    assert all(sample.dbg_ratio_freeze == 0.0 for sample in samples)
    assert samples[-1].motor_hz_out < params.cvt_target_hz