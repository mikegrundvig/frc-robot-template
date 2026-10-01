# Where every value comes from

The robot's code runs on numbers: a frame size, a current limit, how far to trust a camera at 4 m.
Some were measured, some were chosen, and some are still guesses. This register lists every value
in the robot's configuration: which kind it is, where the template's starting number came from, and how to measure
it or why it is what it is. A test keeps the list complete ([below](#keeping-it-complete)).

## Four kinds of value

| Kind | What it is | When it's measured | How |
|---|---|---|---|
| **calibrated** | A number that differs robot to robot: a wheel's radius, a camera's mount, a gyro's scale | On each robot, and again whenever something changes (tread wears, a camera gets bumped) | [docs/calibration.md](calibration.md), from the Pitwall page, or by hand as its row says |
| **characterized** | A number that belongs to a kind of part, or a design: how noisy a camera is at a distance, how late its results arrive, how far a gyro drifts, how a tread grips, a drivetrain design's feedforward | Once per model of part or design, on a bench, a practice floor, or one robot; robots built alike share it; again when the part, its software, or its settings change | [docs/characterization.md](characterization.md) |
| **chosen** | A design decision, not a measurement: a threshold, a margin, a tolerance | Never: it's justified, then checked against logs | Its row says why, and what in a log would make you change it ([below](#checking-a-chosen-value-against-logs)) |
| **as built** | A fact of how the robot is built: a frame size, a gear ratio, a CAN ID | Read off the CAD, a datasheet, or Phoenix Tuner X, once per robot | Its row says where |

## Where the template's numbers came from

A row's **template's starting value** is the number the template ships with: the template's robot
in `Robots.java`, or a record's defaults. Its **source** says where that number came from, never
your robot's: once you measure your own, it lives in `Robots.java` with its date (below). Each
source starts with one of these words, with any detail after it in parentheses.

| Source | Meaning |
|---|---|
| measured | Measured on a robot or a bench. |
| 2026 code | A team's 2026 competition code (2611's, itself built on AdvantageKit's template). |
| top team | A top team's published code. The row names the team. |
| datasheet | A vendor's datasheet, drawing, documentation, or a tool's default. |
| rules | The game manual (2026's, until the next season's kickoff replaces it). |
| math | Worked out from other values, or from statistics. The row says how. |
| simulation | Set until the template's simulator behaved. A real robot will differ. |
| guess | A number the template chose: an estimate or a judgment. Nobody measured it; the row's Why is its reason. |
| unknown | A number inherited from elsewhere (another team's code, a template) whose basis is lost. |
| none | No number to trace: the template's robot doesn't have one (it has no wiring), or it's a name or a seed. |

## The register describes values; `Robots.java` holds a robot's numbers

A row here describes a value: what kind it is and how to measure it. It never holds your robot's
number. Each robot's numbers live in its entry in `Robots.java`, and that's where a measurement is
recorded: the builder call that sets the value, with a comment saying when it was measured and how.
The calibration procedures print their results in exactly this form, ready to paste:

```java
.wheelRadius(Inches.of(1.974)) // measured YYYY-MM-DD
```

Do the same for what you measure by hand:

```java
.mass(Pounds.of(131.2)) // measured YYYY-MM-DD: two scales, bumpers and battery on
```

A number measured for a part rather than a robot (a camera's latency, a tread's friction) goes where
every robot with that part reads it: a shared constant in `Robots.java`, built from the defaults
with only what you measured changed.

```java
private static final VisionConfig VISION =
    VisionConfigBuilder.builder(VisionConfig.DEFAULTS)
        .linearStdDevBaseline(0.012) // characterized YYYY-MM-DD: camera noise, our camera at 1280 x 800
        .distanceExponent(2.1) // characterized YYYY-MM-DD: camera noise
        .build();
```

The defaults in the code (`VisionConfig.DEFAULTS`, `EstimatorConfig.DEFAULTS`,
`LocalizationGate.SCORING`, `SensorErrors.TYPICAL`) are the template's starting points, and this
register says where each came from. When you change one in the code itself, change its row too:
the template's starting value and its source.

## Which records are listed

Every record marked `@ConfigRecord` is either configuration, with a section here, or listed under
[records that aren't configuration](#records-that-arent-configuration). A record is configuration
when its values describe the robot or how its code judges what it reads: numbers someone measured
or chose, fixed while the robot runs except as the pit tunes them. A record built from what the
robot reads or does (an IMU's readings, built every loop) isn't, and neither is one that hands a
mechanism its devices, nor a tool's options for one run (the camera study's). Where such a record
holds assumed numbers, they're listed in a table of their own.

In a configuration record, a value that is itself a configuration record (`RobotConfig.body`), or a
list or optional of them (`RobotConfig.cameras`), has a section of its own rather than a row. And
anything a robot's configuration holds, however deep (anything reached from `RobotConfig`), is
configuration: the test refuses to list it as not.

## Keeping it complete

`ValuesRegisterTest` reads this file and fails, naming exactly what to add or remove, when:

- a configuration record has no section, or one of its values has no row;
- a section or a row names a record or a value the code no longer has;
- a row's kind isn't one of the four, or its source doesn't start with one of the words above, or
  the two legends above list other kinds or sources than the test accepts;
- a value a mechanism tunes live has no row under [tuning limits](#tuning-limits), or a row there
  names one it no longer tunes (the tunings come from `RobotContainer.tunings`, the one list of
  them);
- a switch the running robot publishes has no row under [switches](#switches), or a row there names
  one it doesn't (`ValuesRegisterSimTest`, which builds the robot);
- a number in the code changed and its row didn't: a starting value that doesn't show what the
  template ships (a record's `DEFAULTS`, `LocalizationGate.SCORING`, `SensorErrors.TYPICAL`, or the
  template's robot in `Robots`), or a knob's or a switch's limits that aren't the ones declared.
  A cell shows a number to its own precision (`2.54` shows 2.5376), and only numbers (and a
  switch's on or off) are checked: per-camera values and names are left to people.

So a new value starts in its record, and the test asks for its row. A section is a `###` heading
naming the record in backticks (`` ### `SwerveConfig.Limits` ``, a nested record with its outer
one), followed by a table whose header starts with **Value** and has a **Kind** and a **Source**
column; a tuning's is headed `` ### `drivetrain` tuning ``, with a table whose header starts with
**Knob**. A table under such a heading must be one of those. Write a `|` inside a cell as `\|`. An
example table inside a code block or an HTML comment isn't read. The rest of each row is for
people.

## Checking a chosen value against logs

A chosen value makes decisions, and every decision lands in the log:

- **The cameras':** each image's outcome, with the number that decided it (`Vision/<camera>/Outcomes`:
  "too far: 6.3 m", "ambiguous single tag: 0.25 > 0.20", "turning too fast: 5.4 rad/s"), and the
  estimator's verdict on each measurement (`Vision/<camera>/Decisions`: "accepted", "gate: 20.1 >
  13.8", "too late: older than the history kept").
- **Localization's:** what the estimate rests on and how sure it is (`Localization/Mode`,
  `PositionStdDevMeters`, `HeadingStdDevDegrees`), how far the cameras disagree with it lately
  (`RecentDisagreementMeters`), the recoveries, the collisions, the tilt, the terrain, and whether
  it's good enough to score from (`ReadyToScore`, `NotReadyBecause`).
- **The drivetrain's:** each module's inputs (`Drive/Module<name>/`: currents, velocity, angle), what
  it was asked for (`Drive/SetpointsOptimized`), and the IMUs' (`Drive/Gyro/`, `Drive/BackupGyro/`).

Open a log in AdvantageScope, plot the decision beside what it judged, and ask two questions: how
often did it decide, and was it right when it did? To count, export the log (**File > Export
Data**) to a spreadsheet. A filter that throws out images the other
cameras agreed with is too tight; one that lets through images the estimator then rejects is too
loose. In the simulator, `Sim/PoseErrorMeters` says how far the estimate was from the truth, and a
scenario (`./gradlew run -Pscenario=collision`) shows what a value does when something goes wrong.

Then try another number on the same log. Change it where it's set (in `Robots.java`, or the
defaults) and replay the log through the code (`AKIT_LOG_PATH=logs/<log>.wpilog ./gradlew run
-Preplay`): the replay writes a `_sim` log of what the code decides now, from the same inputs
(the cameras' raw detections included), to compare with the original in AdvantageScope.

Change a chosen value when logs show it deciding wrong, more than once, for a reason you can name;
never from one run. Each row says what to look for.

## Where to start

The values that move the robot most, and are least known today:

1. **Camera latency and time sync** ([procedure](characterization.md#camera-latency-and-time-sync)),
   and how many cameras a coprocessor carries before they slow
   ([procedure](characterization.md#vision-coprocessor-capacity)). Every camera measurement is
   applied at its capture time. A capture time 10 ms off puts every measurement 3 cm off at 3 m/s,
   and nothing else in the code can notice.
2. **Camera noise** ([procedure](characterization.md#camera-noise)). The baseline and the distance
   exponent decide how far the estimate trusts every image, and the gate decides from them which
   images are wrong. The template's are the 2026 code's, for a different camera and coprocessor.
3. **Tread friction and the center of gravity's height**
   ([friction](characterization.md#tread-friction), [CG](calibration.md#the-robots-mass-center-of-gravity-and-inertia)).
   Together they set where the wheels slip, so the drive's current limit, and both are guesses.
4. **Gyro drift and noise** ([procedure](characterization.md#gyro-drift-and-noise)). The gyro owns
   the heading; the estimator should be exactly as sure of it as it deserves.
5. **Odometry error** ([procedure](characterization.md#odometry-error)). How fast the estimate's
   uncertainty grows between camera measurements, so how hard the cameras pull it back.

## The robot

### `RobotConfig`

`frc.robot.RobotConfig`: one robot, in `Robots`. Its other values are records with sections of
their own.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `name` | chosen | `sim` | none | Which robot this is, as its logs name it. One per robot; keep it. |

### `Body`

`frc.robot.lib.config.Body`: the frame, bumpers, and mass. Per robot, in `Robots`. The template's
robot is the largest and heaviest the 2026 rules allowed, with the best case center of gravity.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `frameLength` | as built | 27.5 in | rules (2026: the largest square within R104's 110 in perimeter) | From CAD; check with a tape across the outside of the frame. |
| `frameWidth` | as built | 27.5 in | rules (2026, as `frameLength`) | As `frameLength`, side to side. |
| `height` | as built | 30 in | rules (2026: R104's starting height) | From CAD: the tallest point in the starting configuration. |
| `bumperThickness` | as built | 4 in | rules (2026: R403's most) | From the outside of the frame to the outside of the bumper, cover on. |
| `mass` | calibrated | 149.5 lb | rules (2026: 135 lb with bumpers, R408, and a 14.5 lb battery, R601) | [Weigh it](calibration.md#the-robots-mass-center-of-gravity-and-inertia), bumpers and battery on; again after big changes. |
| `yawMomentOfInertia` | calibrated | 6.7 kg m² | guess (typical) | [From CAD, or SysId's rotation test](calibration.md#the-robots-mass-center-of-gravity-and-inertia). Nothing reads it yet; a simulator that models traction will. |
| `cgHeight` | calibrated | 0.15 m | guess (best case: centered and low) | [From CAD, or a tilt test](calibration.md#the-robots-mass-center-of-gravity-and-inertia). With the tread's friction it sets where the wheels slip, so the drive's current limit. Nothing reads it yet. |

## The drivetrain

### `SwerveConfig`

`frc.robot.lib.config.SwerveConfig`: the swerve drivetrain. Per robot, in `Robots`.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `wheelInset` | as built | 2.625 in | datasheet (SDS MK4i's footprint) | The module's footprint drawing, or CAD: from the outside of the frame to the wheel's center. |

### `SwerveConfig.Modules`

The module hardware, the same at every corner.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `driveMotor` | as built | Kraken X60 | guess (a typical build) | The motor on the module. An `_FOC` model only with a Phoenix Pro license. |
| `steerMotor` | as built | Kraken X60 | guess (a typical build) | As `driveMotor`. |
| `wheelRadius` | calibrated | 2 in | datasheet (a 4 in wheel) | [The wheel radius](calibration.md#the-wheel-radius), before each event: tread wears. |
| `driveGearing` | as built | 6.75 | datasheet (SDS MK4i L2) | The module's datasheet: motor turns per wheel turn. |
| `steerGearing` | as built | 150/7 (21.43) | datasheet (SDS MK4i) | The module's datasheet: motor turns per turn of the module. |
| `wheelCoefficientOfFriction` | characterized | 1.2 | guess (tread on carpet: about 1.0 worn, 1.3 new) | [Tread friction](characterization.md#tread-friction): a pull test, new and worn tread. The simulator's grip; with `cgHeight`, it sets the drive's current limit. |

### `SwerveConfig.Limits`

Current and speed limits, all tunable live within their [tuning limits](#tuning-limits). Check them
in a log of hard driving: each drive motor's currents (`Drive/Module<name>/DriveStatorCurrentAmps`,
`DriveSupplyCurrentAmps`), the battery's voltage, and whether the wheels slipped
(`Localization/TractionLimited`, with `Localization/ResidualMeters`, how far the wheels disagree
with each other).

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `driveStatorCurrent` | chosen | 30 A | math (weight transfer: the front wheels slip near 47 A with the configured tread, μ 1.2, and the 0.15 m center of gravity, about 50 A at μ 1.3, and near 31 A with worn tread, μ 1.0, and a 0.3 m one) | Why: torque past the tread's grip only spins the wheels and drains the battery. Measure friction and the center of gravity first; they set it. Change if the wheels slip at full stick (traction-limited, with the residual rising), or never come near it and the drivers want more push. |
| `driveSupplyCurrent` | chosen | 60 A | guess | Why: caps what each drive motor draws from the battery, so four together can't drag it into a brownout. It steps down: Phoenix lets a motor draw this for at most `driveSupplyLowerTime` (1 s) at a time, then lowers the limit to `driveSupplyLowerCurrent` (40 A) until the motor draws less than that again, so 60 A is for bursts and 40 A for anything longer (the two rows below). A motor draws no more from the battery than flows through it, so with the template's 30 A `driveStatorCurrent` neither limit is ever reached: they matter once the stator limit is raised past them. Set it from the weakest battery's [internal resistance](characterization.md#battery-internal-resistance): the voltage with four drive motors at the limit, about the rest voltage less 4 × limit × resistance, must stay above Systemcore's brownout voltage (still to measure on a real Systemcore) and the vision coprocessor's [ride-through voltage](characterization.md#coprocessor-brownout-ride-through). Change if the battery sags toward either in hard driving. |
| `driveSupplyLowerCurrent` | chosen | 40 A | datasheet (Phoenix 6's default `SupplyCurrentLowerLimit`, written out; the 40 A breaker's rating) | Why: once a drive motor has been held at `driveSupplyCurrent` for `driveSupplyLowerTime`, Phoenix lowers its supply limit to this, until it draws less than this again. Phoenix does it whether the code says so or not (its defaults are 40 A after 1 s), so the template sets it where it shows. At the breaker's rating, a motor can push for as long as the driver holds the stick: an FRC 40 A breaker carries 40 A indefinitely, but the datasheets let one trip at 60 A (150%) and 80 A (200%) as early as 3.9 s and 1.5 s ([Snap Action's AT2](https://www.snapaction.net/assets/img/at2/AT2-Spec-Sheet-Revision-2023.pdf), and its MAXI-size MX5), 9 s and 5 s ([CTRE's](https://ctre.download/files/datasheet/CTRE%20Breaker%20Datasheet.pdf)), or 5 s at 80 A ([REV's](https://www.revrobotics.com/content/docs/REV-11-1860-1863-DS.pdf), which lists no 150%); a breaker already warm trips sooner. Setting it to `driveSupplyCurrent` turns the step-down off, and a long push can then trip the breaker. Check: in a log of a pushing match, a drive motor's `Drive/Module<name>/DriveSupplyCurrentAmps` flat at `driveSupplyCurrent` for a second, then flat at this, is the step-down at work; a motor that drops out for seconds (`Drive/Module<name>/DriveConnected` false, then true again) tripped its breaker anyway. Change: never above the breaker's rating; lower it to a smaller breaker's. |
| `driveSupplyLowerTime` | chosen | 1 s | datasheet (Phoenix 6's default `SupplyCurrentLowerTime`, written out) | Why: longer than any sprint, and well short of a breaker's trip. In the simulator, a full-stick sprint, or one reversed at full speed, holds a drive motor at 60 A or more for at most 0.1 s, even with `driveStatorCurrent` raised to 120 A (and never at the template's 30 A). The simulator's wheels carry less weight than the template's robot, so a real one takes longer: a few tenths of a second. Only a long push, the wheels spinning, holds a motor there for a second. The fastest 40 A breaker trips at 60 A after 3.9 s at the earliest, and at 80 A, the top of `driveSupplyCurrent`'s tuning, after 1.5 s: raise `driveSupplyCurrent` toward 80 A, and shorten this. Check: in a log, how long `Drive/Module<name>/DriveSupplyCurrentAmps` stays at `driveSupplyCurrent` before it drops to `driveSupplyLowerCurrent`: this long. Change if logs show the step-down cutting into sprints, or a breaker tripping though the step-down came. |
| `steerStatorCurrent` | chosen | 40 A | datasheet (Phoenix Tuner X's swerve generator, as the 2026 code kept it) | Why: steering needs little torque, and less current is less heat. Change if modules lag their angle setpoints under load (`Drive/Module<name>/SteerAngle` against `Drive/SetpointsOptimized`). |
| `maxLinearSpeed` | chosen | 4.4 m/s | guess (headroom under the 4.73 m/s free speed) | Why: velocity control needs headroom under the free speed to correct with. Lower it if modules can't reach their setpoints at full stick (`Drive/ModuleVelocities` against `Drive/SetpointsOptimized`). |
| `maxAngularSpeed` | chosen | 1 rot/s | guess (driver-friendly) | Why: fast enough to turn around, slow enough to control. Ask the drivers. At full spin (6.3 rad/s) the cameras' images are thrown out: they're over `VisionConfig.maxYawRateRadiansPerSecond`. |

### `SwerveConfig.Gains`

Drive feedforward and feedback, and steering feedback, all tunable live. Check them in AdvantageScope:
each module's speed against its setpoint (`Drive/ModuleVelocities` against
`Drive/SetpointsOptimized`), and its angle (`Drive/Module<name>/SteerAngle`).

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `driveKs` | characterized | 0 V | simulation (the simulator's wheels have no friction) | [Feedforward](characterization.md#feedforward), per drivetrain design: robots built alike share it. |
| `driveKv` | characterized | 2.54 V/(m/s) | math (ideal: nominal voltage over free speed, `Modules.idealDriveKv()`) | [Feedforward](characterization.md#feedforward), per drivetrain design. A real drivetrain's is a little higher. |
| `driveKp` | chosen | 2.0 V/(m/s) | simulation | Why: corrects what feedforward misses. Tune it live after kS and kV: raise it until speeds track their setpoints, and back off at the first buzz. |
| `steerKp` | chosen | 16 V/rad | 2026 code (kP 100 V/rot, converted) | Why: turns the module to its angle quickly. Tune it live: raise it until a new angle is reached quickly, and back off at overshoot or buzzing. The steering's loop sees a new angle only when the CANcoder sends one, every 10 ms, so it overshoots from about 40 and buzzes past about 77 (at `steerKd` 0.08, in the simulator). |
| `steerKd` | chosen | 0.08 V/(rad/s) | 2026 code (kD 0.5, converted) | Why: damps the steering's overshoot. Raise it if the module overshoots its angle. Too much buzzes too, since it damps on a speed up to 10 ms old: past about 0.2 at `steerKp` 50, in the simulator. |
| `skewLookahead` | chosen | 0.03 s | guess | Why: while the robot spins, its wheels reach their angles late, so a field-relative drive aims at the heading this far ahead. Drive straight while spinning, and tune until the path stays straight. Change it if the path curves the way the robot spins. |

### `SwerveWiring`

`frc.robot.lib.config.SwerveWiring`: where the drivetrain's devices are on the CAN bus, and how they
read. Per robot, in `Robots`. Phoenix Tuner X's swerve generator reports every value here; copy them
across. The template's robot runs only in simulation and has no wiring, so nothing is set today.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `canBus` | as built | none | none | Tuner X: Systemcore's port, or the CANivore's name. |
| `gyroId` | as built | none | none | Tuner X: the Pigeon 2's CAN ID. |
| `gyroYawScale` | calibrated | none (1 until measured) | none | [The gyro's scale](calibration.md#the-gyros-scale), once per gyro, and after replacing or remounting it. |
| `couplingRatio` | as built | none | none | Tuner X's swerve generator, or the module's datasheet: drive motor turns per turn of the module, wheel held still. |
| `systemcoreMount` | as built | none | none | How Systemcore is mounted: flat, or on an edge. Check the backup IMU's pitch and roll against the Pigeon's as the robot tilts: how Systemcore's IMU reads on each mount is still to confirm on a real one. |

### `SwerveWiring.ModuleWiring`

One module's devices.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `driveId` | as built | none | none | Tuner X. |
| `steerId` | as built | none | none | Tuner X. |
| `encoderId` | as built | none | none | Tuner X. |
| `encoderOffset` | calibrated | none | none | Tuner X's swerve generator, with the wheels straight ahead; again whenever a module is rebuilt. |
| `driveInverted` | as built | none | none | Tuner X's swerve generator. |
| `steerInverted` | as built | none | none | Tuner X's swerve generator. |
| `encoderInverted` | as built | none | none | Tuner X's swerve generator. |

## The cameras

### `CameraConfig`

`frc.robot.lib.config.CameraConfig`: one camera on the robot. Per camera, in `Robots`. The
template's four are a 2026 competition robot's, as that team's 2026 code gives them.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `name` | chosen | `front-left`, `front-right`, `back-left`, `back-right` | none | The name PhotonVision knows the camera by. |
| `forward` | calibrated | -7.75 to -11.69 in | 2026 code (a 2026 robot's cameras) | [A camera's mount](calibration.md#a-cameras-mount), starting from CAD; again after mounting, bumping, or remounting it. |
| `left` | calibrated | ±12 to ±12.25 in | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `up` | calibrated | 20.7 to 21 in | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `roll` | calibrated | 0° (one -2°) | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `tiltUp` | calibrated | 8.2° to 12.5° | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `yaw` | calibrated | ±23.2° in front, 181° behind | 2026 code (a 2026 robot's cameras) | As `forward`. |
| `noiseScale` | calibrated | 1 | guess (every camera typical) | How much noisier this camera is than the typical one, whose noise [camera noise](characterization.md#camera-noise) measured: its logged tag residual (`Vision/<camera>/MeanTagResidualMeters`) over the other cameras', once every mount is calibrated. |

### `CameraConfig.Sensor`

The camera itself: per model of camera, at the settings it runs. In `Robots`, shared by the cameras
that use it. The simulator builds its cameras from it.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `widthPixels` | as built | 1280 | guess (a typical AprilTag camera) | The resolution the pipeline runs at, in PhotonVision. |
| `heightPixels` | as built | 800 | guess (a typical AprilTag camera) | As `widthPixels`. |
| `diagonalFieldOfView` | characterized | 80° | guess (typical) | PhotonVision's lens calibration, at the resolution the robot runs, reports it. |
| `frameRate` | characterized | 30 Hz | guess | [Coprocessor capacity](characterization.md#vision-coprocessor-capacity): results per second, with as many cameras as one coprocessor carries on the robot. |
| `latency` | characterized | 30 ms | guess | [Camera latency](characterization.md#camera-latency-and-time-sync): from the image's capture to its result reaching the robot, with as many cameras as one coprocessor carries on the robot ([coprocessor capacity](characterization.md#vision-coprocessor-capacity)). |

## Localization

### `VisionConfig`

`frc.robot.lib.localization.VisionConfig`: which camera results are thrown out, and how far the rest
are trusted. Per robot (`RobotConfig.vision`); every robot starts from `VisionConfig.DEFAULTS`. Five
are tunable live. Each image's outcome, with the number that decided it, is in
`Vision/<camera>/Outcomes`; the estimator's verdict in `Vision/<camera>/Decisions`.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `maxAmbiguity` | chosen | 0.2 | 2026 code (254 used 0.19) | Why: a single tag above it fits two poses about equally well, and the wrong one is far off. Change if "ambiguous single tag" throws out results that agreed with the estimate, or single tags under it are then rejected by the gate. |
| `maxHeightErrorMeters` | chosen | 0.2 m | 2026 code | Why: the robot is on the floor, and tags are where the field says; a solve that misses either by 20 cm is a misread. Check the "off the floor" and "height is off" outcomes against the other cameras. |
| `maxDistanceMeters` | chosen | 6 m | guess | Why: noise grows with distance squared, and beyond some distance a result adds more doubt than it removes. Set it from [camera noise](characterization.md#camera-noise): where a tag's noise passes what odometry drifts in a second or two. Check the "too far" outcomes. |
| `maxYawRateRadiansPerSecond` | chosen | 5 rad/s | top team (254: 5 rad/s over the last 0.3 s) | Why: motion blur, rolling shutter, and a capture time a little off all grow with turning. Check: images just under it should pass the gate as often as the robot's still ones. |
| `linearStdDevBaseline` | characterized | 0.02 m | 2026 code | [Camera noise](characterization.md#camera-noise): position error for one tag at 1 m. Check: in clean driving, the gate should reject about 1 in 1000 measurements; far more and this is too small. |
| `angularStdDevBaseline` | characterized | 0.03 rad | 2026 code | [Camera noise](characterization.md#camera-noise): heading error for one tag at 1 m. |
| `distanceExponent` | characterized | 2 | 2026 code (teams used 2 to 4) | [Camera noise](characterization.md#camera-noise): how fast the error grows with distance. |
| `headingConstrainedFactor` | characterized | 0.5 | top team (6328's factor for the same idea) | [Camera noise](characterization.md#camera-noise): how much less a position solved with the gyro's heading scatters. |
| `speedStdDevFactor` | characterized | 0.25 per m/s of speed | unknown (the template credits top teams, none by name) | [Camera noise, while moving](characterization.md#while-moving). |
| `yawRateStdDevFactor` | characterized | 0.5 per rad/s of turning | unknown (the template credits top teams, none by name) | [Camera noise, while moving](characterization.md#while-moving). |
| `maxHeadingStdDevToConstrain` | chosen | 5° (0.087 rad) | unknown (the template credits top teams, none by name) | Why: a heading-constrained solve is only as good as the heading it's given; until the estimate knows its heading this well (a robot not yet placed), full solves find it. Check how soon after placing the robot its outcomes say "heading-constrained". |
| `minTagsForHeading` | chosen | 2 | guess (the fewest a multi-tag solve has) | Why: one tag's heading is too noisy and too ambiguous to correct the gyro. Raise it if two-tag solves pull the heading around (`Localization/Pose`'s heading stepping against `Drive/Gyro/Yaw`). |
| `maxYawRateForHeading` | chosen | 0.5 rad/s | unknown (the template credits top teams, none by name) | Why: correct the gyro's heading only when the robot is turning slowly, where a capture time a little off barely moves the heading. Check the heading offset (`Localization/Pose`'s heading less `Drive/Gyro/Yaw`): lower it if the offset steps while the robot turns; raise it if the offset is rarely corrected in a match. |
| `maxReprojectionErrorForHeadingPixels` | chosen | 1 px | unknown (the template credits top teams, none by name) | Why: only a solve that fits its tags well corrects the gyro. Check against [camera noise](characterization.md#camera-noise): a still camera's multi-tag solves should fit well under it. |
| `mountToleranceMeters` | chosen | 0.05 m | simulation (calibrated cameras on a season-worn robot showed 0.7 to 1 cm while driving) | Why: a calibrated camera's tags land a little off from noise and the estimate's own error; past this, the camera has moved on its mount, the alert says to recalibrate it, and the excess makes it count for less ([how the robot notices](calibration.md#how-the-robot-notices-a-camera-that-moved)). Check `Vision/<camera>/MountOffsetMeters` on the real robot: freshly calibrated cameras should sit well under it. Raise it if they don't; lower it if a camera known to be knocked, and placing the robot visibly off, stays under it. |
| `latencyMarginSeconds` | chosen | 0.1 s | guess | Why: an image arriving more than this after its camera's usual `latency`, or before it was taken, has a wrong capture time or a coprocessor falling behind, so it's left out. It can't catch a clock off by less than it: 0.1 s is 30 cm at 3 m/s. Set it a little above the spread of `Vision/<camera>/LatencySeconds` once [the capture times are checked](characterization.md#camera-latency-and-time-sync). Raise it if images in time are left out (`Vision/<camera>/LeftOutOfStep`); lower it toward that spread if the check finds capture times off by more than a few milliseconds. |

### `EstimatorConfig`

`frc.robot.lib.localization.EstimatorConfig`: how the pose estimator weighs odometry, the gyro, and
the cameras. Per robot (`RobotConfig.estimator`); every robot starts from
`EstimatorConfig.DEFAULTS`, which the code calls starting points, not measurements.

The estimator's uncertainty should match its error. In the simulator, the
[consistency study](testing.md#is-the-estimator-honest-the-consistency-study) checks it over many
runs (`./gradlew consistencyStudy`), and one run's log compares `Localization/PositionStdDevMeters`
with `Sim/PoseErrorMeters`; on a robot, compare it with how far the cameras move the estimate when
they see tags again after a stretch without. An estimate often off by
three times its uncertainty is too sure of itself, and its gate rejects good measurements; one never
off by half of it is too unsure, and the cameras pull it around.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `translationStdDevPerMeter` | characterized | 0.03 m | guess | [Odometry error](characterization.md#odometry-error), per drivetrain design and tread: robots built alike share it. |
| `translationStdDevPerRadian` | characterized | 0.02 m | guess | [Odometry error](characterization.md#odometry-error): spinning in place. |
| `residualScale` | chosen | 1 | guess (all of it) | Why: wheels that disagree with each other put the robot's travel in doubt by about as much. Lower it if the uncertainty balloons in clean driving (`Localization/ResidualMeters` high, the cameras finding little to correct). |
| `tractionLimitedSlipFraction` | chosen | 0.25 | guess | Why: wheels at the edge of their grip slip some of the time. Check how far the cameras correct the estimate after stretches of `Localization/TractionLimited`. |
| `tiltStartDegrees` | chosen | 5° | guess | Why: a small tilt is carpet and bumpers, not wheels off the floor. Check: `Localization/TiltDegrees` stays under it in flat driving. |
| `tiltZeroDegrees` | chosen | 25° | top team (6328) | Why: by this tilt, some wheels are off the floor. Check with a [terrain crossing](calibration.md#a-terrain-crossing). |
| `slipCorrelationSeconds` | chosen | 0.5 s | guess | Why: a slip (a push, a launch) keeps its direction for a while, so its errors add up rather than average out. Check: after a push or a launch, the cameras' correction should fall inside the uncertainty the estimate claimed (`PositionStdDevMeters`). Lengthen it if corrections often land outside; shorten it if they're always far inside. |
| `collisionStdDevMeters` | chosen | 0.25 m | guess | Why: a hit moves the robot further than its wheels see. Check: after logged hits (`Localization/Collision`), how far the cameras moved the estimate; it should cover the largest. |
| `collisionWindowSeconds` | chosen | 0.25 s | guess | Why: one hit jolts for several loops; count it once. Check: one hit's jolts span less than it. |
| `handlingStdDevPerSqrtSecond` | chosen | 0.02 m | guess (the idea: 254's loose trust while disabled) | Why: a disabled robot gets carried, and its wheels can't tell, so the cameras should lead. Check: placed and disabled, the estimate settles on the cameras before the match. |
| `headingDriftDegreesPerMinute` | characterized | 0.5° | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise): the gyro's worst drift, standing still. |
| `headingStdDevDegreesPerTurn` | characterized | 1° | guess (an uncalibrated gyro's scale is off by a few tenths of a percent: the simulator's season-worn one reads 0.3% high, 1.1° a turn; the consistency study draws the gyro's scale from this, so it can't say what it should be) | [The gyro's scale, repeated](characterization.md#repeatability-of-the-gyros-scale): how far its scale may still be off after calibrating it; until it's calibrated, how far an uncalibrated one may be. The estimator takes it as the same error every turn, so five turns are five times as far off: lower it once calibrated. |
| `backupHeadingStdDevDegreesPerTurn` | characterized | 2° | guess (an uncalibrated IMU: the simulator's season-worn backup reads 0.5% high, 1.8° a turn) | [The backup IMU's scale](characterization.md#repeatability-of-the-gyros-scale), from the gyro-scale procedure's spins. The procedure calibrates the gyro alone: lowering `headingStdDevDegreesPerTurn` after it leaves this as it is. |
| `headingNoiseDegreesPerTurn` | characterized | 0.1° | guess (small beside the scale's error; the simulator's IMUs have none, so the study can't check it) | [The gyro's scale, repeated](characterization.md#repeatability-of-the-gyros-scale): the spread of each turn's error about the five turns' straight line, once the scale is taken out. |
| `wheelHeadingStdDevDegreesPerTurn` | characterized | 10° | guess | [Odometry error](characterization.md#odometry-error): heading from the wheels against the gyro's, spinning at driving speed. Used only while the gyro is lost. |
| `cameraTurnStdDevDegrees` | characterized | 0.25° | guess (the mount procedure placed a knocked camera within 0.1° in the simulator; more for the lens, and margin) | What calibrating a camera's mount leaves of its turn: [a camera's mount](calibration.md#a-cameras-mount), run five times from the same spots; the spread of the yaw it finds. Too small, and the estimator grows surer than the cameras know, standing still (the [consistency study](testing.md#is-the-estimator-honest-the-consistency-study): `--mount-error-degrees=1` puts standing at a NEES of 2.9 with this at 0.25°, 6 without it). |
| `cameraShiftStdDevMeters` | characterized | 0.01 m | guess (the field's tags within half an inch of the layout, the mount's shift a few millimeters; checked in the consistency study, whose tags sit within that tolerance) | How far a camera's positions sit off the same way image after image, beyond its turn: standing on surveyed spots of a practice field, each camera's average position over 10 s against the spot; their root mean square, after taking out what the camera's turn explains (its tags' distance times the yaw it's off). At an event, a field built off its layout raises it. |
| `cameraShiftStdDevDegrees` | characterized | 2° | simulation (the consistency study, tags off by `cameraShiftStdDevMeters`: two tags a third of a meter apart, each 1 cm off, turn a multi-tag heading by about 2°; at 1°, standing after the gyro-scale procedure the heading grew sure of a wrong value, a heading NEES of 2.05 and 4.9% of loops outside the ellipse, against 0.95 and 0.1% at 2°) | How far a camera's multi-tag headings sit off the same way while it sees the same tags, beyond its turn: on surveyed spots of a practice field, facing known headings, each camera's average multi-tag heading over 10 s against the truth; their root mean square, less the camera's turn. A field surveyed with WPIcal, or tags that move together on one panel, need less. |
| `cameraShiftCorrelationMeters` | chosen | 1 m | guess | Why: about a meter on, a camera sees other tags, or the same ones from another angle, and what its positions share is mostly new. Check in the consistency study: lengthen it if the driving runs' NEES sits above its band, shorten it if well below. |
| `cameraShiftCorrelationDegrees` | chosen | 45° | guess | Why: as `cameraShiftCorrelationMeters`, for turning: an eighth of a turn brings other tags into view. |
| `historySeconds` | chosen | 1.5 s | guess (254 kept 1 s) | Why: a camera measurement must arrive within it to be applied; several times any camera's [latency](characterization.md#camera-latency-and-time-sync). Check for "too late" in `Decisions`. |
| `futureToleranceSeconds` | chosen | 0.05 s | guess | Why: clocks differ a little, but a measurement stamped this far past the newest odometry has a wrong capture time. Check for "from the future" in `Decisions` ([time sync](characterization.md#camera-latency-and-time-sync)). |
| `visionTimeoutSeconds` | chosen | 1 s | guess | Why: after a second without an accepted measurement, the estimate is dead reckoning, and says so. Check: `Localization/Mode` reads DEAD_RECKONING only where the cameras are really blind. |
| `disagreementWindowSeconds` | chosen | 0.5 s | guess | Why: how far back "lately" reaches for `RecentDisagreementMeters`, which the scoring gate reads. Lengthen it if `NotReadyBecause` flickers on one stray measurement; shorten it if the robot stays not ready long after the cameras agree again. |
| `gateChiSquaredPosition` | chosen | 13.816 | math (chi-squared, 2 degrees of freedom: 99.9% of good measurements pass) | Why: rejects measurements its noise model says are almost certainly wrong. A good noise model rejects about 1 in 1000 good measurements; far more "gate:" rejections in clean driving mean the uncertainties are too small, not that the gate is too tight. |
| `gateChiSquaredPose` | chosen | 16.266 | math (3 degrees of freedom, 99.9%) | As `gateChiSquaredPosition`, for a measurement with heading. |
| `gateChiSquaredAxis` | chosen | 10.828 | math (1 degree of freedom, 99.9%) | As `gateChiSquaredPosition`, for a landmark. |
| `recoveryMeasurements` | chosen | 3 | guess | Why: three rejected measurements that agree with each other outvote odometry; with fewer, one bad camera could. Check: `Localization/Recoveries` counts only real knocks (a hit, a push, a bad placement). |
| `recoveryWindowSeconds` | chosen | 1 s | guess | Why: those measurements must be recent, so they describe the same moment. As `recoveryMeasurements`. |
| `recoveryAgreementMeters` | chosen | 0.25 m | guess | Why: measurements this close agree, given camera noise at range. Keep it above the noise at the distances the cameras see tags. |
| `recoveryStdDevMeters` | chosen | 0.25 m | guess | Why: after a jump, the estimate is that unsure, so the next measurements settle it. Change if the measurements right after a recovery are rejected (too small), or the estimate wanders for seconds after one (too large). |
| `recoveryAgreementDegrees` | chosen | 5° | guess | Why: rejected measurements whose headings agree within this, and not with the gyro, mean the heading is wrong (a bad reset, a robot placed backwards). Check: heading recoveries happen only after a real wrong heading; lower it if one follows a single camera's noise. |
| `recoveryHeadingStdDevDegrees` | chosen | 2° | guess | Why: as `recoveryStdDevMeters`, for heading. Change as `recoveryStdDevMeters`. |
| `lostAfterRejections` | chosen | 2 | guess | Why: two rejections in a row with none accepted, and the estimate says it's lost. Check: LOST in `Localization/Mode` away from real knocks means it's too low, or the noise model too sure. |
| `lostPositionStdDevMeters` | chosen | 1 m | guess | Why: after a heading recovery, the position was worked out with the wrong heading, so it may be far off. Change if, after a heading recovery, the cameras find the position further off than this. |

### `LocalizationGate`

`frc.robot.lib.localization.LocalizationGate`: whether an estimate is good enough to score from;
the robot uses `LocalizationGate.SCORING`. Check `Localization/ReadyToScore` and `NotReadyBecause`
in a match log: ready whenever the robot is lined up with tags in view, and never while the estimate
is off.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `maxVisionAgeSeconds` | chosen | 0.5 s | top team (254) | Why: odometry drifts after the cameras last checked it. Change if `NotReadyBecause` blames an old measurement while the robot is lined up with tags in view (the cameras are slower than it), or shots taken just under it miss. |
| `maxDisagreementMeters` | chosen | 0.2 m | top team (254) | Why: cameras that just disagreed by more haven't settled the estimate. Lower it toward the task's tolerance once [camera noise](characterization.md#camera-noise) says the cameras do better. |
| `maxPositionStdDevMeters` | chosen | 0.15 m | guess | Why: the scoring task's tolerance. Set it from the game: how far off a shot or a placement may land. |
| `maxTiltDegrees` | chosen | 5° | guess (the idea: 1678 and 6328 wouldn't shoot tilted) | Why: a tilted robot aims off, and its cameras solve worse. Change if shots taken just under it miss, or the robot reads over it on level floor. |

### `ImuCrossCheck.Settings`

`frc.robot.lib.localization.ImuCrossCheck.Settings`: how closely the gyro and the backup IMU must
agree before an alert says one is failing; the robot uses `Settings.DEFAULTS`. Check
`Localization/ImuDisagreementDegrees`: well inside the tolerance in normal driving, and over it only
for a real fault.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `windowSeconds` | chosen | 1 s | guess | Why: long enough to see two IMUs turn apart, short enough to catch a frozen one in a second. Change if a frozen gyro (the `gyro-frozen` scenario) takes too long to flag, or hard turns set off the alert. |
| `toleranceDegrees` | chosen | 3° | guess | Why: above what the two drift and scatter apart in the window ([gyro drift and noise](characterization.md#gyro-drift-and-noise), for both). Change if the alert comes on in normal driving. |
| `scaleTolerance` | chosen | 0.05 | guess | Why: above the two IMUs' difference in scale; the gyro's is calibrated, the backup's isn't ([the backup's scale](characterization.md#repeatability-of-the-gyros-scale)). Change if fast spins set off the alert with both IMUs healthy. |

### `TerrainCrossing.Settings`

`frc.robot.lib.localization.TerrainCrossing.Settings`: what counts as climbing, level, and one
feature; the robot uses `Settings.DEFAULTS`. Check them with a
[terrain crossing](calibration.md#a-terrain-crossing) on a real robot, and set them again at kickoff from
the new field's ramps.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `steepDegrees` | chosen | 10° | top team (1678) | Why: under the ramps' slope (2026's: 15°), over anything flat driving reads. Lower it if `TerrainCrossings` misses a crossing; raise it if it counts one in flat driving. |
| `levelDegrees` | chosen | 5° | top team (1678) | Why: what a flat floor reads, bumpers and carpet included. Raise it if the robot never reads level after a crossing. |
| `levelSeconds` | chosen | 0.1 s | guess | Why: five loops level is off the feature, not a wobble on its top. Lengthen it if a wobble on a feature's top ends a crossing early. |
| `topSeconds` | chosen | 1.5 s | guess | Why: time to cross a feature's flat top; longer is two features, or a stop on top. Lengthen it if a wide feature's crossing isn't counted. |

## The simulator

### `SensorErrors`

`frc.robot.sim.SensorErrors`: how the simulated robot's sensors differ from the truth.
`SensorErrors.TYPICAL` is a robot partway through a season (the simulator and its scenarios use it);
`NONE` is exact, for tests of exact numbers. The code calls TYPICAL's values plausible estimates:
replace them with a real robot's once it's calibrated.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `wheelRadiusScale` | characterized | 0.985, 0.99, 0.98, 0.988 | guess | The spread of each wheel's radius in [the wheel radius](calibration.md#the-wheel-radius)'s report, partway through a season, over the radius configured. |
| `steerOffsetDegrees` | characterized | 0.5, -0.3, 0, 0.4 | guess | How far modules point off straight after their offsets are set: square the wheels against a straightedge, and read each module's angle (`Drive/Module<name>/SteerAngle`). |
| `cameraMountErrors` | characterized | none | guess (calibrated mounts stay put) | How far a mount moves between calibrations: [a camera's mount](calibration.md#a-cameras-mount)'s "before" residual, run again weeks later. Scenarios, and the Pitwall page, knock a mount while the simulator runs (`frc.robot.sim.Faults`). |
| `tagErrors` | characterized | none | none (the simulator's field is exactly its layout) | How far a field's tags sit from its layout: a survey of the field (WPIcal). The consistency study draws them within the field's half-inch tolerance; the Pitwall page moves a tag while the simulator runs (`frc.robot.sim.Faults`). |
| `seed` | chosen | 1 | none | Any number: fixed, so a run repeats exactly. |

### `SensorErrors.GyroErrors`

One gyro's errors. TYPICAL gives the gyro and the backup IMU each their own; EXACT has none.

| Value | Kind | Template's starting value | Source | How to measure it, or why |
|---|---|---|---|---|
| `scale` | characterized | 1.003 (backup 1.005) | guess | [The gyro's scale](calibration.md#the-gyros-scale) before it's calibrated (one over its first `gyroYawScale`), and [the backup's](characterization.md#repeatability-of-the-gyros-scale). |
| `driftDegreesPerMinute` | characterized | 0.2 (backup 1.0) | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise), a typical run. The backup's is unknown until measured. |
| `noiseDegrees` | characterized | 0.02 (backup 0.05) | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise), a typical run. |

## Tuning limits

How far the pit may take each value a mechanism tunes live, declared with the value in the
mechanism's `tuning`, and listed together in `RobotContainer.tunings`. All are chosen: wide enough to tune in, narrow enough that a slip of a finger
can't hurt the robot. The record's own checks still apply inside them. Change a limit when tuning on
a robot keeps wanting to go past it, after asking why.

### `drivetrain` tuning

`Drive.tuning`.

| Knob | Limits | Source | Why |
|---|---|---|---|
| `gains.driveKs` | 0 to 1 V | guess | Static friction takes a few tenths of a volt; past a volt, the robot creeps. |
| `gains.driveKv` | 0 to 5 V/(m/s) | guess | Twice the ideal 2.54: room for a real drivetrain's losses, not for a unit mistake. |
| `gains.driveKp` | 0 to 20 | guess | Ten times the template's: room to find where it buzzes. |
| `gains.steerKp` | 0 to 100 | guess | About six times the template's. |
| `gains.steerKd` | 0 to 5 | guess | Far above the template's; damping this high only adds noise. |
| `gains.skewLookahead` | 0 to 0.1 s | guess | Five loops; past it, the robot steers ahead of itself. |
| `limits.driveStatorCurrent` | 10 to 120 A | datasheet (120 A: Phoenix 6's default stator limit) | Up to Phoenix 6's own default; under 10 A the robot barely moves. |
| `limits.driveSupplyCurrent` | 10 to 80 A | guess | Room above the template's 60 A; four motors drawing more brown out a battery. |
| `limits.driveSupplyLowerCurrent` | 10 to 40 A | datasheet (40 A: the drive motors' breakers' rating) | Never above what the breaker carries indefinitely. |
| `limits.driveSupplyLowerTime` | 0.1 to 1 s | datasheet (the fastest 40 A breaker, Snap Action's, can trip after 1.5 s at 80 A, the top of `limits.driveSupplyCurrent`) | At any supply limit in its range, the step-down comes at least half a second before the fastest breaker can trip (a warm breaker trips sooner than its datasheet's cold figures). Under 0.1 s it would cut into every hard launch. |
| `limits.steerStatorCurrent` | 5 to 80 A | guess | Steering needs little; the top is for finding out how little. |
| `limits.maxLinearSpeed` | 0.5 to 6 m/s | guess | Slow enough for a first drive; the record refuses anything past the free speed. |
| `limits.maxAngularSpeed` | 0.1 to 3 rot/s | guess | Slow enough for a first drive, and three times the template's. |

### `vision` tuning

`Vision.tuning`.

| Knob | Limits | Source | Why |
|---|---|---|---|
| `maxAmbiguity` | 0 to 1 | math (an ambiguity is a ratio from 0 to 1) | Its whole range. |
| `maxHeightErrorMeters` | 0 to 1 m | guess | A meter off the floor is no robot. |
| `linearStdDevBaseline` | 0.001 to 1 m | guess | Never zero, which would trust a camera completely. |
| `angularStdDevBaseline` | 0.001 to 1 rad | guess | As `linearStdDevBaseline`. |
| `distanceExponent` | 0 to 4 | top team (teams used 2 to 4) | The range teams used, and flat. |
| `mountToleranceMeters` | 0.01 to 0.5 m | guess | From tighter than a calibrated camera can hold to a camera far off its mount. |
| `latencyMarginSeconds` | 0.01 to 1 s | guess | From tighter than a camera's own jitter to a second, still inside the estimator's 1.5 s of history. |

### Experiment rigs

A rig is a mechanism as Pitwall's experiments run it. Its gains are values its mechanism already
tunes (the steering's are the drivetrain's `gains.steerKp` and `gains.steerKd`, above), so their
limits are checked there. A rig's own limits are another kind of number: not how far a value may
be tuned, but where an experiment may send the mechanism, so the setpoints of every test motion stay
inside them. Rigs are built from a running mechanism and registered in `RobotContainer`, so the
test doesn't read them: add a row here when you add a rig or an experiment.

| Rig or experiment | Numbers | Kind | Source | Why |
|---|---|---|---|---|
| The steering rig (`Drive.steeringRig`): where an experiment may point the wheels | -180° to 180° | chosen | guess | A module turns all the way around, so any angle is safe; the limits keep a test motion within one turn, as the wheel's angle is scored. |
| The steering's experiment (`RobotContainer`): "Steer to 90° and back" | 0° to 90° and back, 0.8 s at each, after a 0.5 s lead-in | chosen | guess | A quarter turn shows rise, overshoot, and settling; 0.8 s is several times the steering's settling (122 ms in the simulator), so each step settles before the next. |

## Switches

The drive team's switches: WPILib tunables a mechanism publishes for itself, not configuration.
Each keeps its setting into a match, within its limits, with an alert naming each one changed since
the robot started. `ValuesRegisterSimTest` builds the robot and checks every switch it publishes has
a row here, with its limits and its starting value; a switch published for each camera is written
with `<camera>` in its path.

| Switch | Limits | Template's starting value | Source | Why |
|---|---|---|---|---|
| `Vision/autoStartHoldSeconds` | 0 to 30 s | 0 s | guess (6328 held theirs 2 s) | Why: holds the cameras' measurements back as autonomous starts, trusting the placed start while the robot launches. Off, since the estimator gates every measurement against the placed start anyway; 30 s is longer than any autonomous period has been. Turn it on if logs show the cameras pulling the estimate off its start in autonomous's first seconds. |
| `Vision/<camera>/enabled` | on or off | on | none | Why: leaves one camera out, fogged over, knocked loose, or blinded by lights, while the others carry on. |

## The camera study's assumptions

The camera placement study ([docs/camera-placement.md](camera-placement.md)) scores where cameras
could go by simulating what they'd see, so it assumes how the cameras and the robot err. Its
options (`CameraStudyConfig`) are listed under the records that aren't configuration, since they're
a run's choices; these are the assumptions inside them, and the ranges a robustness run draws
from. The test doesn't check this list. Measuring the cameras ([camera noise](characterization.md#camera-noise))
replaces the nominal values, and narrows the ranges.

| Assumption | Template's starting value | Kind | Source | How to measure it, or why |
|---|---|---|---|---|
| Tag corner noise, nominal | 0.25 px average, 0.08 px spread | characterized | datasheet (`VisionSim`'s, PhotonVision's simulation example) | [Camera noise](characterization.md#camera-noise). |
| How far tags are seen, nominal | 7 m | characterized | guess (`VisionSim`'s) | [Camera noise](characterization.md#camera-noise): the farthest a tag is found in most images. |
| The gyro's heading error, nominal | 0.25°, one standard deviation | characterized | guess | The estimator's heading uncertainty while cameras correct it (`Localization/HeadingStdDevDegrees`), checked in the simulator against `Sim/HeadingErrorDegrees`. |
| Each camera off its mount, nominal | none | characterized | guess (calibrated mounts) | [A camera's mount](calibration.md#a-cameras-mount): its result's "how sure" after calibrating. |
| Corner noise, robustness range | 0.1 to 0.6 px | chosen | guess | A sharp, still global-shutter image locates corners to about a tenth of a pixel; defocus, a softer lens, or blur, half a pixel or more ([the study's reasons](camera-placement.md#robustness)). |
| Sight, robustness range | 5 to 7 m | chosen | guess | Where detection gives out for a field-size tag at the camera's resolution, with exposure and blur ([the study's reasons](camera-placement.md#robustness)). |
| Gyro heading error, robustness range | 0.1° to 0.5° | chosen | guess | A few tenths of a degree while cameras correct it; half a degree after a hit or a stretch without two tags ([the study's reasons](camera-placement.md#robustness)). |
| Mount error, robustness range | 0 to 5 mm and 0° to 1° per axis | chosen | guess | A mount checked by calibration lands within a few millimeters and a fraction of a degree; a bumped or uncalibrated one, a degree or more ([the study's reasons](camera-placement.md#robustness)). |

## The consistency study's assumptions

The estimator consistency study ([docs/testing.md](testing.md#is-the-estimator-honest-the-consistency-study))
checks the estimator's uncertainty against the simulator's truth, so it assumes how a calibrated
robot errs: by default, as the estimator's configuration claims, so that an honest estimator reads
a NEES near its number of dimensions. Its options are a run's choices, not configuration; these are
the assumptions inside them. The test doesn't check this list.

| Assumption | Template's starting value | Kind | Source | How to measure it, or why |
|---|---|---|---|---|
| The robot's wear | `SensorErrors.TYPICAL`, but each IMU's scale drawn per seed | characterized | guess (the simulator's season-worn robot) | Its rows [above](#sensorerrors). |
| Each camera off its mount | Gaussian, 3 mm along and `cameraTurnStdDevDegrees` about each axis, drawn for each seed | calibrated | guess (3 mm: what the mount procedure leaves); math (the turn: as the configuration claims) | [A camera's mount](calibration.md#a-cameras-mount): the spread of what five runs from the same spots find. Wider with `--mount-error-mm` and `--mount-error-degrees`. |
| Each tag off the field's layout | Gaussian, `cameraShiftStdDevMeters` along each axis, drawn for each seed | characterized | math (as the configuration claims) | A survey of the field (WPIcal). Wider with `--tag-error-mm`. |
| The gyro's scale | Gaussian, off by `headingStdDevDegreesPerTurn` / 360, drawn for each seed | characterized | math (as the configuration claims) | [The gyro's scale, repeated](characterization.md#repeatability-of-the-gyros-scale). |
| The backup IMU's scale | Gaussian, off by 0.5%, drawn for each seed | characterized | guess (`SensorErrors.TYPICAL`'s backup) | [The backup IMU's scale](characterization.md#repeatability-of-the-gyros-scale). |
| The vision noise model | half the configured one (`--noise-scale=0.5`) | characterized | simulation (at 1, the study reads a position NEES near 0.7; at half, near 2) | [Camera noise](characterization.md#camera-noise): how noisy the robot's own cameras are against the 2026 model. |

## Numbers still written in code

These aren't configuration yet, and the test doesn't check this list. A number worth measuring
belongs in a configuration record, where the test makes the register cover it; until it moves, it's
listed here. Other numbers in code (deadbands, the calibration procedures' own settings) are the
code's choices, explained beside them.

| Where | Number | Kind | Source | How to measure it, or why |
|---|---|---|---|---|
| `VisionSim`: the farthest tag a simulated camera detects | 7 m | characterized | guess | [Camera noise](characterization.md#camera-noise): the farthest a tag is found in most images. |
| `VisionSim`: tag corner noise | 0.25 px average, 0.08 px spread | characterized | datasheet (PhotonVision's simulation example) | [Camera noise](characterization.md#camera-noise): how far the corners scatter, standing still. |
| `VisionSim`: latency's spread | 5 ms | characterized | datasheet (PhotonVision's simulation example) | [Camera latency](characterization.md#camera-latency-and-time-sync). |
| `PhoenixDrivetrain`: how often each module's CANcoder sends its position and velocity (the simulator's steering hears it as often) | 100 Hz | chosen | datasheet (Phoenix 6's default for a CANcoder's position) | Why: without Phoenix Pro, the steering's loop sees a new angle only this often, which sets how high its gains can go. Faster allows higher gains, for more traffic on the CAN bus: check the bus's load before raising it. Check that a robot's CANcoders send this often: Phoenix Tuner X's plot of a CANcoder's position shows a new sample every 10 ms, and in a log of a module turning, its odometry angles (`Drive/Module<name>/OdometrySteerAnglesRad`) change only every 10 ms, the same angle two or three readings running. |
| `ModuleIOSim`: how often the steering's motor controller runs its loop | 1 ms | as built | datasheet (CTRE's Phoenix 6 documentation: a TalonFX's closed loop runs at 1 kHz) | A fact of the motor controller. |
| `ModuleIOSim`: drive and steering inertia | 0.025 and 0.004 kg m² | characterized | datasheet (Phoenix Tuner X's simulation defaults, as AdvantageKit's template uses) | [Feedforward](characterization.md#feedforward): SysId on one module. |
| `Localization`: a jolt that's a collision | 20 m/s² | chosen | top team (6800) | Why: hard driving stays near 1 g. Check: the IMU's acceleration (`Drive/Gyro/AccelerationXMetersPerSecSq`, `AccelerationYMetersPerSecSq`) stays well under it in hard driving, and goes over it in hits. |
| `Localization`: a tilt past a bump's slope still counted as the ground | 3° | calibrated | guess | [A terrain crossing](calibration.md#a-terrain-crossing), on a real robot: bumpers, suspension, and the IMU's mount. |
| `Localization`: how closely a crest places the robot | 5 cm | calibrated | guess | [A terrain crossing](calibration.md#a-terrain-crossing), on a real robot. |
| `Drive`: a wheel this far short of its speed is at its grip's edge | 0.5 m/s | chosen | guess (the idea: 581 and 1678) | Why: a wheel that can't keep up is slipping or stalled. Check: `Localization/TractionLimited` turns on in pushing matches, and not in clean driving. |
| `Vision`: how far back "lately" reaches for turning | 0.3 s | chosen | top team (254) | Why: an image taken just after a fast turn is still blurred, and its capture time matters most. |
| `Vision`: how many of a camera's latest tags its mount is judged by | 500 | chosen | simulation (a knock putting tags 6 cm off alerts after 417 tags, 10 cm after 251) | Why: enough for noise and the estimate's own error to average out, few enough that a knock shows within seconds of seeing tags. Check: after a real knock, how long until the alert. |
| `Vision`: how recently another camera must have agreed for a camera's tags to count toward its mount | 2 s | chosen | simulation (half a second left out cameras facing other ways, which see tags at other moments) | Why: the other cameras hold the pose a camera is judged against. |

## Records that aren't configuration

| Record | Why it isn't configuration |
|---|---|
| `ImuReadings` | What one IMU read in one loop. |
| `DrivetrainReadings` | What the drivetrain read in one loop, for localization. |
| `RobotStatus` | What the drive team's dashboard shows, built each loop. |
| `DriveHardware` | The drivetrain's devices, handed to it: hardware, not numbers. |
| `Scenario` | A named situation for the simulator. Its numbers are what a test needs, written in `Scenarios`. |
| `CameraStudyConfig` | The camera placement study's options for one run (how finely it samples the field, how many trials and threads, its seed), set on the command line. Nothing on a robot reads it. Its assumptions about how cameras and the robot err are listed under [the camera study's assumptions](#the-camera-studys-assumptions). |
