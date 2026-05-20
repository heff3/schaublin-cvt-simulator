from spindle_plant_model import SpindlePlantModel, SpindlePlantParams, SpindlePlantState
from spindle_supervisor_model import SupervisorOutputs


def build_control(*, motor_cmd_rpm: float, backgear_state: bool = False, cvt_increase: bool = False, cvt_decrease: bool = False) -> SupervisorOutputs:
    return SupervisorOutputs(
        motor_cmd_rpm=motor_cmd_rpm,
        vfd_brake=False,
        backgear_on=backgear_state,
        backgear_off=not backgear_state,
        cvt_increase=cvt_increase,
        cvt_decrease=cvt_decrease,
        backgear_state=backgear_state,
        fault=False,
        closed_loop=True,
        ratio_out=1.3,
        motor_hz_out=motor_cmd_rpm / 30.0,
        active_kp=0.0,
        active_ki=0.0,
        dbg_error_rpm=0.0,
        dbg_integrator=0.0,
        dbg_cvt_wait=0.0,
        dbg_ol_stable=0.0,
        dbg_state=3.0,
        dbg_ratio_freeze=0.0,
        dbg_motor_hz_err=0.0,
        dbg_cvt_pulse_calc=0.0,
        dbg_fb_rpm_abs=0.0,
    )


def test_cvt_increase_reduces_ratio_and_raises_spindle_speed():
    plant = SpindlePlantModel(
        params=SpindlePlantParams(motor_time_constant=0.0, cvt_ratio_rate_per_sec=0.5),
        state=SpindlePlantState(motor_rpm=1200.0, spindle_rpm=1200.0 / 1.3, cvt_ratio_high_gear=1.3),
    )

    before = plant.step(build_control(motor_cmd_rpm=1200.0), dt=0.1)
    after = plant.step(build_control(motor_cmd_rpm=1200.0, cvt_increase=True), dt=0.1)

    assert after.cvt_ratio_high_gear < before.cvt_ratio_high_gear
    assert after.spindle_rpm > before.spindle_rpm


def test_cvt_direction_fault_can_be_inverted_for_regressions():
    plant = SpindlePlantModel(
        params=SpindlePlantParams(
            motor_time_constant=0.0,
            cvt_ratio_rate_per_sec=0.5,
            cvt_direction_inverted=True,
        ),
        state=SpindlePlantState(motor_rpm=1200.0, spindle_rpm=1200.0 / 1.3, cvt_ratio_high_gear=1.3),
    )

    before = plant.step(build_control(motor_cmd_rpm=1200.0), dt=0.1)
    after = plant.step(build_control(motor_cmd_rpm=1200.0, cvt_increase=True), dt=0.1)

    assert after.cvt_ratio_high_gear > before.cvt_ratio_high_gear
    assert after.spindle_rpm < before.spindle_rpm