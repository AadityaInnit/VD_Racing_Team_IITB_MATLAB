# Transient skidpad experiment

This is an independent prototype. It does not edit or call the existing
`skidpad_laptime_ggv.m`, GGV generator, or equilibrium solver files.

## Run in MATLAB

```matlab
run('/Users/aaditya/Desktop/VD_Racing_Team/Transient_Skidpad/run_transient_skidpad.m')
```

Or, from this folder:

```matlab
c = transient_skidpad_config();
c.makePlots = false;
o = transient_skidpad(c);
```

`run_transient_skidpad.m` shows the plots and saves `transient_skidpad_results.mat`.
The configuration file is the place to edit parameters. All planning, driver,
trim, and plant functions are local functions in `transient_skidpad.m`.

The package contains a snapshot of the current GGV in `data/` and the original
Pacejka fit in `private/`. It needs MATLAB and Optimization Toolbox, but does
not depend on other files elsewhere in the project.

## What is integrated

The time-domain ODE states are

```
[Vx, Vy, yaw rate r, track progress s, lateral error ey,
 body heading error epsi, actual steering angle delta]
```

The body dynamics are

```
dVx/dt = FX/m + r*Vy
dVy/dt = FY/m - r*Vx
dr/dt  = Mz/Iz
```

`FX` and `FY` include drag. Tire forces use the instantaneous axle contact
velocities `(Vx, Vy+a*r)` and `(Vx, Vy-b*r)`, the existing nonlinear lateral
Pacejka fit, and an ellipse approximation for combined slip. The fit's negative
cornering stiffness requires slip angle = contact-velocity heading minus steer.
Front wheel forces are rotated into the body frame. Actual forces determine
all accelerations; the speed controller's requested acceleration is not imposed.

The reference is the original circle, with curvature `kappa = 1/R`. Its errors
are integrated using exact planar Frenet kinematics:

```
ds/dt     = (Vx*cos(epsi) - Vy*sin(epsi))/(1-kappa*ey)
dey/dt    = Vx*sin(epsi) + Vy*cos(epsi)
depsi/dt  = r - kappa*ds/dt
```

The actual CG trajectory is reconstructed from `s` and `ey`; `ey` is never reset.
Neither yaw rate nor sideslip is reset to a reference equilibrium. Actual
trajectory curvature is `(r + betaDot)/V`, while the reference remains `1/R`.
Time integration starts at rest, avoiding division by path speed.

The GGV planner retains the corner cap, forward acceleration, reverse braking,
minimum speed profile, and station time sum. The dynamic driver follows this
plan using steering feedforward plus course/path/yaw feedback, and force
feedforward plus speed feedback. Steering has a time constant, rate limit, and
angle limit. Steady plant trims supply only feedforward steering and flying-start
initial conditions; their solver is not called during dynamic integration.

## Useful experiments

Flying start:

```matlab
c.startMode = "flying";
o = transient_skidpad(c);
```

A less aggressive target:

```matlab
c.speedScale = 0.9;
```

Brake and accelerate again on the same circle:

```matlab
c.startMode = "flying";
c.speedScale = 0.9;
c.speedSchedule = [0 1; .25 1; .45 .6; .70 .6; .85 1; 1 1];
o = transient_skidpad(c);
```

Explore yaw inertia or perturb initial yaw rate:

```matlab
c.vehicle.Iz = 150; % replace with a measured value when available
c.yawInertiaSource = "User-supplied value";
c.initialYawRateOffset = -0.2; % rad/s
```

Yaw inertia defaults to `m*a*b`, an equivalent two-point axle mass approximation,
not a measured chassis inertia. Changing it affects the transient response.
Changing other vehicle parameters requires regenerating and replacing the
package's GGV snapshot.

## Outputs and failure handling

`o` contains body velocities, sideslip/rate, yaw rate/acceleration, forces and
moment, actual trajectory/curvature, reference speed, path error, steering,
wheel normal loads, tire utilization, and load-transfer iteration residuals.

A lap finishes when reference-track progress reaches `2*pi*R`. Track-boundary,
sideslip-limit, reversal, or time-limit failures return `completed=false` and
`lapTime=NaN`; `t(end)` is elapsed simulation time, not a successful lap time.
Completion means crossing the lap finish within the configured corridor, not
following a mathematically exact circle or proving minimum lap time.

## Model assumptions

- Flat road, bicycle axle kinematics, four individual tire normal loads.
- Roll/pitch load transfer is algebraic, not a suspension transient model.
  Normal loads are solved iteratively from actual ground-level tire forces.
  Drag acts at the CG; its drive-force compensation contributes to pitch transfer.
- Drive force is limited by wheel power and tire capacity. There are no wheel
  rotational, drivetrain torque-rise, or slip-ratio dynamics.
- Tire forces respond instantaneously to slip angle. No tire relaxation lengths
  or aligning moments are currently included.
- Combined slip retains an ellipse approximation, not measured full MF weighting.
- Lateral tire force is smoothly reduced near rest, with a regularized low-speed
  slip-angle calculation. This launch approximation is configurable.
- The inherited capacity table extends to 2000 N, although the longitudinal fit's
  stated fitted range is 200--1100 N. `tireFitRangeExceeded` reports load-range or
  slip-angle-range violations. The nominal run can exceed this range; successful
  completion does not validate those extrapolated forces against measurements.
- `wheelLiftPredicted` flags near-zero wheel loads. Clipped transfer conserves
  axle load, but meaningful wheel-lift/roll behavior needs a roll dynamics model.
- Steering gains, response parameters, and estimated inertia are experimental.
  The output is a driver-controlled feasibility simulation, not an optimal-control
  lap-time solution or a measured-vehicle validation.

## Tests

```matlab
results = runtests('test_transient_skidpad.m');
assertSuccess(results)
```

Eight tests cover standing launch, flying equilibrium, numerical Newton-equation
checks, trajectory/course kinematics, braking yaw dynamics, inertia sensitivity,
failure reporting, and integration convergence.

## Saved demonstration (7 October 2026)

Default full-GGV target, standing start, estimated `Iz = 196.0976 kg*m^2`:

- Planned station time: **5.05448 s**.
- Dynamically simulated lap: **5.08183 s**.
- Maximum lateral path error: **0.0910 m**.
- Sideslip: approximately **-1.432 to +4.068 degrees**.
- Tire-fit range diagnostic: **exceeded**; wheel loads approximately **82--1804 N**.

The saved MAT file and overview/dynamics PNGs illustrate this run. The eight
regression tests pass, and Code Analyzer reports no issues in the new code.

## Parameter sweeps

Edit and run `run_parameter_sweep.m`. Its default example sweeps yaw inertia
from 100 to 300 kg*m^2 in increments of 50. Output CSV, MAT, and summary PNG
are written to `sweep_output/`.

```matlab
% Discover all numeric settings, including matrix entries:
list_sweep_parameters()

% Any numeric scalar field; colon, linspace, or explicit vector:
spec = struct('parameter',"vehicle.ClA",'values',linspace(2,6,9));
study = sweep_transient_skidpad(spec,OutputDirectory="my_sweep");

% Every combination of two (or more) parameters:
spec(1) = struct('parameter',"vehicle.m",'values',[300 330 360]);
spec(2) = struct('parameter',"steerTimeConstant",'values',[.03 .05 .10]);
study = sweep_transient_skidpad(spec,StoreTrajectories=true);

% Indexed entries, without eval or executable expressions:
spec = struct('parameter',"speedSchedule(2,2)",'values',.6:.1:1);
```

Examples include `vehicle.Iz`, `vehicle.m`, `vehicle.wb`, `vehicle.rwb`,
`vehicle.ClA`, `vehicle.CdA`, `vehicle.aero_bal_f`, `vehicle.h_cg`,
`vehicle.track_f`, `vehicle.track_r`, `vehicle.tire.P`, `vehicle.tire.gamma_f`,
`radius`, `powerLimit`, `speedScale`, and all numeric driver/integration settings.
Angles use radians, consistent with the configuration file. Categorical string
and logical settings are set in the base configuration, rather than swept over
numeric ranges.

Sweeping wheelbase/rear weight fraction updates CG axle distances automatically.
Sweeping axle distances instead updates wheelbase/rear weight fraction. Mixing
both representations in one sweep is rejected. Estimated yaw inertia follows
`m*a*b` unless explicitly swept, or `UpdateEstimatedInertia=false` is supplied.
Explicitly labelled measured inertia remains fixed. The effective inertia is
included in the results table.

**Reference policy:** a sweep deliberately uses the package's fixed nominal GGV
while changing the dynamic plant. Vehicle/drivetrain mismatch is identified by
`UsesNominalGGV`. Pressure/camber changes rebuild the plant tire-capacity tables
from the copied Pacejka fit. Other vehicle changes alter forces, loads, inertia,
and trim in the plant; they do not regenerate the GGV acceleration surfaces.
The longitudinal fit itself has no pressure/camber correction.

This compares tracking/feasibility and achieved times against a nominal target;
it does not find each modified vehicle's independently optimized lap time.
Regenerated per-vehicle GGV surfaces would be a separate, more expensive study.
Ordinary standalone runs retain the mismatch check unless `allowNominalGGV=true`
is explicitly set. Existing saved configurations remain supported.

Each combination records lap time, planned time, path error, maximum sideslip,
yaw acceleration, tire utilization, wheel loads, inertia, runtime, completion,
tire-fit flags, and exception text. Infeasible or invalid cases retain NaN lap
times and do not stop subsequent combinations. No failed case is ranked as a
completed lap. One-dimensional plots show parameter versus lap time/path error;
two-dimensional plots color combinations by lap time; higher-dimensional studies
use combination numbers and retain all parameter columns in the CSV.

`MaxRuns` defaults to 1000 and guards accidental Cartesian explosions. Individual
run plots are disabled during sweeps. `StoreTrajectories=true` retains full run
outputs in the saved study; the default stores only the summary. A sweep remains
sequential for reproducibility and requires no Parallel Computing Toolbox.

Regression tests:

```matlab
r = runtests({'test_parameter_sweep.m','test_transient_skidpad.m'});
assertSuccess(r)
```
