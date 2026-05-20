# CVT Spindle Simulation Harness

This workspace contains two things:

1. The original LinuxCNC spindle control artifacts for the Schaublin 125 CVT and backgear drivetrain.
2. A host-side Python simulation harness used to reproduce, study, and eventually fix the current spindle control issues in software before changing machine behavior.

The immediate focus is spindle speed control with a variable-ratio CVT, a two-speed backgear, and a supervisor component that blends open-loop startup, closed-loop PI control, ratio learning, and CVT pulse commands.

## Current Status

The Python harness is set up and green under `uv`.

Current test coverage includes:

- direct supervisor-model checks for open-loop stability gating and closed-loop ratio learning behavior
- direct plant-model checks for normal and inverted CVT direction semantics
- a coupled controller-plus-plant regression that reproduces the logged backgear ratio-corruption signature

At this point the harness intentionally models the current behavior, including the suspected bug path. The goal is to make failures reproducible first, then change behavior with regression coverage in place.

## Source Files

### LinuxCNC control files

- `spindle_supervisor_2026-05-17.comp`
  - LinuxCNC component that owns the spindle supervisor state machine and control logic.
- `spindle_2026-05-17-3.hal`
  - HAL wiring for the supervisor, VFD, encoder, CVT outputs, and backgear outputs.
- `schaublin_125-CNC_2026-05-17-2.ini`
  - Machine and spindle tuning values. The Python harness reads the `[SPINDLE_0]` section directly.
- `spindle_log_report_2026-05-17.txt`
  - Diagnostic report from the current machine behavior. This is the main reference for the first regression targets.

### Python simulation files

- `spindle_supervisor_model.py`
  - Host-side step model of the supervisor behavior.
  - Mirrors the current logic around state transitions, ratio learning, PI control, CVT pulse timing, and debug outputs.
- `spindle_plant_model.py`
  - Minimal drivetrain plant model.
  - Simulates motor response, CVT ratio movement, backgear multiplication, and simple fault injection.
- `spindle_simulation.py`
  - Coupled controller-plus-plant runner for multi-step scenarios.
- `tests/`
  - Focused regression tests for the supervisor model, plant model, and coupled scenarios.

## Why This Exists

The current diagnostics point to several control problems that are easier to analyze in software than on the machine:

- ratio learning is effectively blocked or corrupted under real feedback velocity conditions
- large single-sample `ratio_out` jumps destabilize the closed-loop behavior
- CVT direction and pulse behavior can be wrong or inconsistent
- saturation and backgear logic need repeatable regression coverage before retuning

The Python harness makes those behaviors executable in small deterministic tests.

## Getting Started

### Prerequisites

- `uv`
- Python `>=3.9`

### Install and run tests

From this directory:

```bash
uv run --group dev pytest -q
```

That command is the current reference check for the Python harness.

## Example Scenario

This example runs a short coupled controller-plus-plant simulation and injects a forced encoder drop, which is close to the current ratio-corruption investigation.

```python
from spindle_plant_model import SpindlePlantModel, SpindlePlantParams, SpindlePlantState
from spindle_simulation import run_coupled_simulation
from spindle_supervisor_model import SpindleSupervisorModel, SupervisorInputs, SupervisorParams, SupervisorState

params = SupervisorParams.from_ini("schaublin_125-CNC_2026-05-17-2.ini")

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

for sample in samples:
  print(
    sample.step_index,
    round(sample.ratio_out, 3),
    round(sample.fb_vel, 1),
    round(sample.motor_hz_out, 1),
    round(sample.plant_spindle_rpm, 1),
  )
```

Expected shape with the current model: `ratio_out` climbs rapidly, `fb_vel` stays high, and `motor_hz_out` rises toward the upper VFD range before `dbg_ratio_freeze` eventually engages.

## Test Inventory

### Supervisor model tests

`tests/test_spindle_supervisor_model.py`

Covers:

- closed-loop ratio learning still updating under high `fb_vel`
- open-loop stability timer reset under the default `OL_STABLE_BAND`

### Plant model tests

`tests/test_spindle_plant_model.py`

Covers:

- CVT increase lowering ratio and increasing spindle speed
- inverted CVT direction for regression and fault modeling

### Coupled scenario tests

`tests/test_spindle_closed_loop_scenarios.py`

Covers:

- backgear closed-loop ratio corruption under forced encoder drop conditions
- a reproduction of the jump pattern seen in the diagnostic report around `ratio_out` moving from about `7.989` upward while `fb_vel` remains high

## Modeling Notes

- The harness reads spindle tuning directly from `[SPINDLE_0]` in the existing LinuxCNC INI file rather than duplicating those values.
- The INI reader is intentionally narrow and tolerant because the machine INI is not a clean generic config file.
- The supervisor model preserves output-like state such as prior `motor_cmd_rpm`, because the LinuxCNC component effectively reuses those values across servo cycles.
- The plant model is deliberately simple. It is meant to support regression tests, not to be a full physics model of the machine.

## Recommended Workflow

1. Reproduce a machine issue in a Python regression first.
2. Change behavior in the Python model until the regression expresses the intended fix.
3. Port the same logic into `spindle_supervisor_2026-05-17.comp`.
4. Re-run the Python harness and then validate at the LinuxCNC layer.

## Current Next Step

The most direct next change is to tighten the ratio-learning gate in the supervisor model so high `fb_vel` prevents ratio updates during the corruption scenario. Once that behavior is covered and stable in Python, the same fix can be applied to the LinuxCNC component.
