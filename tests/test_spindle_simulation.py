from pathlib import Path

from spindle_plant_model import SpindlePlantModel, SpindlePlantParams, SpindlePlantState
from spindle_simulation import (
    CoupledSimulationSession,
    RealTimeSimulationScheduler,
    SimulationCommand,
)
from spindle_supervisor_model import SpindleSupervisorModel, SupervisorParams


ROOT = Path(__file__).resolve().parents[1]
INI_PATH = ROOT / "schaublin_125-CNC_2026-05-17-2.ini"


def load_params() -> SupervisorParams:
    return SupervisorParams.from_ini(INI_PATH)


def build_session(*, dt: float = 0.1, history_limit: int | None = None) -> CoupledSimulationSession:
    controller = SpindleSupervisorModel(load_params())
    plant = SpindlePlantModel(
        SpindlePlantParams(motor_time_constant=0.0, encoder_noise_rpm=5.0),
        SpindlePlantState(motor_rpm=60.0, spindle_rpm=40.0, cvt_ratio_high_gear=1.5),
    )
    return CoupledSimulationSession(controller, plant, dt=dt, history_limit=history_limit)


def test_session_step_command_tracks_time_and_feedback():
    session = build_session(dt=0.1, history_limit=4)

    sample = session.step_command(SimulationCommand(cmd_rpm=120.0, enable=True))

    assert sample.step_index == 0
    assert sample.time_s == 0.0
    assert sample.measured_fb_rpm == 45.0
    assert session.step_index == 1
    assert session.time_s == 0.1
    assert session.last_sample == sample
    assert session.history == (sample,)


def test_session_history_is_bounded():
    session = build_session(dt=0.05, history_limit=2)
    command = SimulationCommand(cmd_rpm=180.0, enable=True)

    for _ in range(3):
        session.step_command(command)

    assert [sample.step_index for sample in session.history] == [1, 2]


class FakeTime:
    def __init__(self, *values: float):
        self._values = iter(values)

    def __call__(self) -> float:
        return next(self._values)


def test_realtime_scheduler_caps_catch_up_and_tracks_drops():
    session = build_session(dt=0.1, history_limit=8)
    scheduler = RealTimeSimulationScheduler(
        session,
        lambda: SimulationCommand(cmd_rpm=150.0, enable=True),
        time_source=FakeTime(0.0, 0.35),
        max_steps_per_update=2,
    )

    scheduler.start()
    result = scheduler.advance()

    assert len(result.samples) == 2
    assert [sample.step_index for sample in result.samples] == [0, 1]
    assert result.dropped_steps == 1
    assert scheduler.total_dropped_steps == 1
    assert session.step_index == 2