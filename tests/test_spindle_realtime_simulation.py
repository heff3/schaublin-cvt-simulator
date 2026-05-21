from pathlib import Path

from spindle_plant_model import SpindlePlantModel, SpindlePlantParams, SpindlePlantState
from spindle_realtime_simulation import MANUAL_CVT_BUTTON_TICKS, OperatorPanelState
from spindle_simulation import CoupledSimulationSession
from spindle_supervisor_model import SpindleSupervisorModel, SupervisorParams, SupervisorState


ROOT = Path(__file__).resolve().parents[1]
INI_PATH = ROOT / "schaublin_125-CNC_2026-05-17-2.ini"


def load_params() -> SupervisorParams:
    return SupervisorParams.from_ini(INI_PATH)


def test_operator_panel_holds_manual_cvt_press_for_required_ticks():
    panel = OperatorPanelState()

    panel.pulse_cvt_increase()
    commands = [panel.next_command() for _ in range(MANUAL_CVT_BUTTON_TICKS + 1)]

    assert [command.panel_cvt_increase for command in commands] == [True] * MANUAL_CVT_BUTTON_TICKS + [False]
    assert [command.panel_cvt_decrease for command in commands] == [False] * (MANUAL_CVT_BUTTON_TICKS + 1)


def test_single_manual_cvt_keypress_reaches_supervisor_press_gate():
    params = load_params()
    controller = SpindleSupervisorModel(
        params,
        SupervisorState(
            state=3,
            ratio_filt=1.3,
            ratio_init_done=True,
            gear_init_done=True,
            bg_state=False,
            motor_cmd_rpm=420.0,
            prev_fb_abs=320.0,
        ),
    )
    plant = SpindlePlantModel(
        SpindlePlantParams(motor_time_constant=0.0),
        SpindlePlantState(
            motor_rpm=420.0,
            spindle_rpm=320.0,
            cvt_ratio_high_gear=1.3,
        ),
    )
    session = CoupledSimulationSession(controller, plant, dt=0.001, history_limit=16)
    panel = OperatorPanelState(cmd_rpm=320.0, enable=True, spindle_reverse=True)
    initial_ratio = plant.state.cvt_ratio_high_gear

    panel.pulse_cvt_increase()
    for _ in range(MANUAL_CVT_BUTTON_TICKS):
        session.step_command(panel.next_command())

    assert session.last_control is not None
    assert session.last_control.cvt_increase is True
    assert session.last_control.cvt_decrease is False
    assert plant.state.cvt_ratio_high_gear < initial_ratio
