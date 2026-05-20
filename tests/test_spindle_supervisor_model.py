from pathlib import Path

from spindle_supervisor_model import SpindleSupervisorModel, SupervisorInputs, SupervisorParams, SupervisorState


ROOT = Path(__file__).resolve().parents[1]
INI_PATH = ROOT / "schaublin_125-CNC_2026-05-17-2.ini"


def load_model(state: SupervisorState | None = None) -> SpindleSupervisorModel:
    params = SupervisorParams.from_ini(INI_PATH)
    return SpindleSupervisorModel(params=params, state=state)


def test_closed_loop_ratio_learning_ignores_high_feedback_velocity():
    model = load_model(
        SupervisorState(
            state=3,
            ratio_filt=1.0,
            ratio_init_done=True,
            gear_init_done=True,
            motor_cmd_rpm=1500.0,
            prev_fb_abs=1000.0,
        )
    )

    outputs = model.step(
        SupervisorInputs(
            cmd_rpm=1200.0,
            fb_rpm=600.0,
            enable=True,
        ),
        dt=0.001,
    )

    assert model.state.fb_vel > model.params.ol_stable_band
    assert outputs.ratio_out > 1.0


def test_open_loop_stability_timer_resets_under_default_band_with_fast_feedback_change():
    model = load_model(
        SupervisorState(
            state=2,
            ratio_filt=1.3,
            ratio_init_done=True,
            gear_init_done=True,
            prev_fb_abs=0.0,
        )
    )

    outputs = model.step(
        SupervisorInputs(
            cmd_rpm=335.0,
            fb_rpm=150.0,
            enable=True,
        ),
        dt=0.001,
    )

    assert model.state.fb_vel > model.params.ol_stable_band
    assert outputs.dbg_ol_stable == 0.0