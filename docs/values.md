# Where every value comes from

The robot's code runs on numbers: a frame size, a current limit, how far to trust a camera at 4 m.
Some were measured, some were chosen, and some are still guesses. This register lists every value
in the robot's configuration: which kind it is, where today's number came from, and how to measure
it or why it is what it is. A test keeps the list complete ([below](#keeping-it-complete)).

## Four kinds of value

| Kind | What it is | When it's measured | How |
|---|---|---|---|
| **calibrated** | A number that differs robot to robot: a wheel's radius, a camera's mount, a gyro's scale | On each robot, and again whenever something changes (tread wears, a camera gets bumped) | [docs/calibration.md](calibration.md), from the Pitwall page, or by hand as its row says |
| **characterized** | A number that belongs to a kind of part: how noisy a camera is at a distance, how late its results arrive, how far a gyro drifts, how a tread grips | Once per model of part, on a bench or a practice floor; again when the part, its software, or its settings change | [docs/characterization.md](characterization.md) |
| **chosen** | A design decision, not a measurement: a threshold, a margin, a tolerance | Never: it's justified, then checked against logs | Its row says why, and what in a log would make you change it |
| **as built** | A fact of how the robot is built: a frame size, a gear ratio, a CAN ID | Read off the CAD, a datasheet, or Phoenix Tuner X, once per robot | Its row says where |

## Where today's number came from

Each row's source starts with one of these words, with any detail after it in parentheses.

| Source | Meaning |
|---|---|
| measured | Measured on a robot or a bench. |
| 2026 code | A team's 2026 competition code (2611's, itself built on AdvantageKit's template). |
| top team | A top team's published code. The row names the team. |
| datasheet | A vendor's datasheet, drawing, documentation, or a tool's default. |
| rules | The game manual (2026's, until the next season's kickoff replaces it). |
| math | Worked out from other values, or from statistics. The row says how. |
| simulation | Set until the template's simulator behaved. A real robot will differ. |
| guess | The template's own estimate or judgment. The row gives its reason; nobody measured it. |
| unknown | Nothing records where the number came from. |
| none | No number to trace: the template's robot doesn't have one (it has no wiring), or it's a name or a seed. |

IDs such as M1 and S13 name the open question a value waits on, in the team's list of open
questions.

## The register describes values; `Robots.java` holds a robot's numbers

A row here describes a value: what kind it is and how to measure it. It never holds your robot's
number. Each robot's numbers live in its entry in `Robots.java`, and that's where a measurement is
recorded: the builder call that sets the value, with a comment saying when it was measured and how.
The calibration procedures print their results in exactly this form, ready to paste:

```java
.wheelRadius(Inches.of(1.974)) // measured 2027-02-14
```

Do the same for what you measure by hand:

```java
.mass(Pounds.of(131.2)) // measured 2027-02-14: two scales, bumpers and battery on
```

A number measured for a part rather than a robot (a camera's latency, a tread's friction) goes where
every robot with that part reads it: a shared constant in `Robots.java`, built from the defaults
with only what you measured changed.

```java
private static final VisionConfig VISION =
    VisionConfigBuilder.builder(VisionConfig.DEFAULTS)
        .linearStdDevBaseline(0.012) // characterized 2027-01-20: camera noise, our camera at 1280 x 800
        .distanceExponent(2.1) // characterized 2027-01-20: camera noise
        .build();
```

The defaults in the code (`VisionConfig.DEFAULTS`, `EstimatorConfig.DEFAULTS`,
`LocalizationGate.SCORING`, `SensorErrors.TYPICAL`) are the template's starting points, and this
register says where each came from. When you change one in the code itself, change its row too:
today's number and its source.

## Which records are listed

Every record marked `@ConfigRecord` is either configuration, with a section here, or listed under
[records that aren't configuration](#records-that-arent-configuration). A record is configuration
when its values describe the robot or how its code judges what it reads: numbers someone measured
or chose, fixed while the robot runs except as the pit tunes them. A record built from what the
robot reads or does (an IMU's readings, built every loop) isn't, and neither is one that hands a
mechanism its devices.

In a configuration record, a value that is itself a configuration record (`RobotConfig.body`), or a
list or optional of them (`RobotConfig.cameras`), has a section of its own rather than a row.

## Keeping it complete

`ValuesRegisterTest` reads this file and fails, naming exactly what to add or remove, when:

- a configuration record has no section, or one of its values has no row;
- a section or a row names a record or a value the code no longer has;
- a row's kind isn't one of the four, or its source doesn't start with one of the words above;
- a value a mechanism tunes live has no row under [tuning limits](#tuning-limits), or a row there
  names one it no longer tunes.

So a new value starts in its record, and the test asks for its row. A section is a `###` heading
naming the record in backticks (`` ### `SwerveConfig.Limits` ``, a nested record with its outer
one), followed by a table whose header starts with **Value** and has a **Kind** and a **Source**
column. The test reads only a row's name, kind, and source; the rest is for people.

## Where to start

The values that move the robot most, and are least known today:

1. **Camera latency and time sync** ([procedure](characterization.md#camera-latency-and-time-sync)).
   Every camera measurement is applied at its capture time. A capture time 10 ms off puts every
   measurement 3 cm off at 3 m/s, and nothing else in the code can notice.
2. **Camera noise** ([procedure](characterization.md#camera-noise)). The baseline and the distance
   exponent decide how far the estimate trusts every image, and the gate decides from them which
   images are wrong. Today's are the 2026 code's, for a different camera and coprocessor.
3. **Tread friction and the center of gravity's height**
   ([friction](characterization.md#tread-friction), [CG](characterization.md#a-robots-mass-center-of-gravity-and-inertia)).
   Together they set where the wheels slip, so the drive's current limit, and both are guesses.
4. **Gyro drift and noise** ([procedure](characterization.md#gyro-drift-and-noise)). The gyro owns
   the heading; the estimator should be exactly as sure of it as it deserves.
5. **Odometry error** ([procedure](characterization.md#odometry-error)). How fast the estimate's
   uncertainty grows between camera measurements, so how hard the cameras pull it back.

## The robot

### `RobotConfig`

`frc.robot.RobotConfig`: one robot, in `Robots`. Its other values are records with sections of
their own.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `name` | chosen | `sim` | none | Which robot this is, as its logs name it. One per robot; keep it. |

### `Body`

`frc.robot.lib.config.Body`: the frame, bumpers, and mass. Per robot, in `Robots`. The template's
robot is the largest and heaviest the 2026 rules allowed, with the best case center of gravity.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `frameLength` | as built | 27.5 in | rules (2026: the largest square within R104's 110 in perimeter) | From CAD; check with a tape across the outside of the frame. |
| `frameWidth` | as built | 27.5 in | rules (2026, as `frameLength`) | As `frameLength`, side to side. |
| `height` | as built | 30 in | guess (typical) | From CAD: the tallest point in the starting configuration. |
| `bumperThickness` | as built | 4 in | rules (2026: R403's most) | From the outside of the frame to the outside of the bumper, cover on. |
| `mass` | calibrated | 149.5 lb | rules (2026: 135 lb with bumpers, R408, and a 14.5 lb battery, R601) | [Weigh it](characterization.md#a-robots-mass-center-of-gravity-and-inertia), bumpers and battery on; again after big changes. |
| `yawMomentOfInertia` | calibrated | 6.7 kg m² | guess (typical) | [From CAD, or SysId's rotation test](characterization.md#a-robots-mass-center-of-gravity-and-inertia) (M5). Nothing reads it yet; a simulator that models traction will. |
| `cgHeight` | calibrated | 0.15 m | guess (best case: centered and low) | [From CAD, or a tilt test](characterization.md#a-robots-mass-center-of-gravity-and-inertia) (M3). With the tread's friction it sets where the wheels slip, so the drive's current limit. Nothing reads it yet. |

## The drivetrain

### `SwerveConfig`

`frc.robot.lib.config.SwerveConfig`: the swerve drivetrain. Per robot, in `Robots`.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `wheelInset` | as built | 2.625 in | datasheet (SDS MK4i's footprint) | The module's footprint drawing, or CAD: from the outside of the frame to the wheel's center. |

### `SwerveConfig.Modules`

The module hardware, the same at every corner.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `driveMotor` | as built | Kraken X60 | guess (a typical build) | The motor on the module. An `_FOC` model only with a Phoenix Pro license. |
| `steerMotor` | as built | Kraken X60 | guess (a typical build) | As `driveMotor`. |
| `wheelRadius` | calibrated | 2 in | datasheet (a 4 in wheel) | [The wheel radius](calibration.md#the-wheel-radius), before each event: tread wears. |
| `driveGearing` | as built | 6.75 | datasheet (SDS MK4i L2) | The module's datasheet: motor turns per wheel turn. |
| `steerGearing` | as built | 150/7 | datasheet (SDS MK4i) | The module's datasheet: motor turns per turn of the module. |
| `wheelCoefficientOfFriction` | characterized | 1.2 | guess (tread on carpet: about 1.0 worn, 1.3 new) | [Tread friction](characterization.md#tread-friction): a pull test, new and worn tread (M1). The simulator's grip; with `cgHeight`, it sets the drive's current limit. |

### `SwerveConfig.Limits`

Current and speed limits, all tunable live within their [tuning limits](#tuning-limits). Check them
in a log of hard driving: each drive motor's currents (`Drive/Module<name>/DriveStatorCurrentAmps`,
`DriveSupplyCurrentAmps`), the battery's voltage, and whether the wheels slipped
(`Localization/TractionLimited`, with `Localization/ResidualMeters`, how far the wheels disagree
with each other).

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `driveStatorCurrent` | chosen | 30 A | math (weight transfer: slip near 31 A with worn tread and a 0.3 m center of gravity, 47 A at best) | Why: torque past the tread's grip only spins the wheels and drains the battery. Measure friction and the center of gravity first; they set it. Change if the wheels slip at full stick (traction-limited, with the residual rising), or never come near it and the drivers want more push. |
| `driveSupplyCurrent` | chosen | 60 A | unknown | Why: caps what each drive motor draws from the battery, so four together can't drag it into a brownout. Change if the battery sags toward the brownout voltage in hard driving (S5). |
| `steerStatorCurrent` | chosen | 40 A | guess (steering needs little torque) | Why: steering needs little torque, and less current is less heat. Change if modules lag their angle setpoints under load (`Drive/Module<name>/SteerAngle` against `Drive/SetpointsOptimized`). |
| `maxLinearSpeed` | chosen | 4.4 m/s | guess (headroom under the 4.73 m/s free speed) | Why: velocity control needs headroom under the free speed to correct with. Lower it if modules can't reach their setpoints at full stick (`Drive/ModuleVelocities` against `Drive/SetpointsOptimized`). |
| `maxAngularSpeed` | chosen | 1 rotation/s | guess (driver-friendly) | Why: fast enough to turn around, slow enough to control. Ask the drivers. At full spin (6.3 rad/s) the cameras' images are thrown out: they're over `VisionConfig.maxYawRateRadiansPerSecond`. |

### `SwerveConfig.Gains`

Drive feedforward and feedback, and steering feedback, all tunable live. Check them in AdvantageScope:
each module's speed against its setpoint (`Drive/ModuleVelocities` against
`Drive/SetpointsOptimized`), and its angle (`Drive/Module<name>/SteerAngle`).

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `driveKs` | characterized | 0 V | simulation (the simulator's wheels have no friction) | [Feedforward](characterization.md#feedforward), per drivetrain. |
| `driveKv` | characterized | 2.54 V per m/s | math (ideal: nominal voltage over free speed, `Modules.idealDriveKv()`) | [Feedforward](characterization.md#feedforward). A real drivetrain's is a little higher. |
| `driveKp` | chosen | 2.0 V per m/s | simulation | Why: corrects what feedforward misses. Tune it live after kS and kV: raise it until speeds track their setpoints, and back off at the first buzz. |
| `steerKp` | chosen | 16 V per radian | 2026 code (kP 100 V per rotation, converted) | Why: turns the module to its angle quickly. Tune it live: raise it until a new angle is reached quickly, and back off at overshoot or buzzing. |
| `steerKd` | chosen | 0.08 V per rad/s | 2026 code (kD 0.5, converted) | Why: damps the steering's overshoot. Raise it if the module overshoots its angle. |
| `skewLookahead` | chosen | 0.03 s | unknown | Why: while the robot spins, its wheels reach their angles late, so a field-relative drive aims at the heading this far ahead. Drive straight while spinning, and tune until the path stays straight. |

### `SwerveWiring`

`frc.robot.lib.config.SwerveWiring`: where the drivetrain's devices are on the CAN bus, and how they
read. Per robot, in `Robots`. Phoenix Tuner X's swerve generator reports every value here; copy them
across. The template's robot runs only in simulation and has no wiring, so nothing is set today.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `canBus` | as built | none | none | Tuner X: Systemcore's port, or the CANivore's name. |
| `gyroId` | as built | none | none | Tuner X: the Pigeon 2's CAN ID. |
| `gyroYawScale` | calibrated | none (1 until measured) | none | [The gyro's scale](calibration.md#the-gyros-scale), once per gyro, and after replacing or remounting it. |
| `couplingRatio` | as built | none | none | Tuner X's swerve generator, or the module's datasheet: drive motor turns per turn of the module, wheel held still. |
| `systemcoreMount` | as built | none | none | How Systemcore is mounted: flat, or on an edge. Check the backup IMU's pitch and roll against the Pigeon's as the robot tilts (S13). |

### `SwerveWiring.ModuleWiring`

One module's devices.

| Value | Kind | Today | Source | How to measure it, or why |
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

`frc.robot.lib.config.CameraConfig`: one camera on the robot. Per camera, in `Robots`. Today's four
are a 2026 competition robot's, as that team measured them.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `name` | chosen | `front-left`, `front-right`, `back-left`, `back-right` | none | The name PhotonVision knows the camera by. |
| `forward` | calibrated | -7.75 to -11.69 in | measured (a 2026 robot's cameras) | [A camera's mount](calibration.md#a-cameras-mount), starting from CAD; again after mounting, bumping, or remounting it. |
| `left` | calibrated | ±12 to ±12.25 in | measured (a 2026 robot's cameras) | As `forward`. |
| `up` | calibrated | 20.7 to 21 in | measured (a 2026 robot's cameras) | As `forward`. |
| `roll` | calibrated | 0° (one -2°) | measured (a 2026 robot's cameras) | As `forward`. |
| `tiltUp` | calibrated | 8.2° to 12.5° | measured (a 2026 robot's cameras) | As `forward`. |
| `yaw` | calibrated | ±23.2° in front, 181° behind | measured (a 2026 robot's cameras) | As `forward`. |
| `noiseScale` | calibrated | 1 | guess (every camera typical) | How much noisier this camera is than the typical one, whose noise [camera noise](characterization.md#camera-noise) measured: its logged tag residual (`Vision/<camera>/MeanTagResidualMeters`) over the other cameras', once every mount is calibrated. |

### `CameraConfig.Sensor`

The camera itself: per model of camera, at the settings it runs. In `Robots`, shared by the cameras
that use it. The simulator builds its cameras from it.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `widthPixels` | as built | 1280 | guess (a typical AprilTag camera) | The resolution the pipeline runs at, in PhotonVision. |
| `heightPixels` | as built | 800 | guess (a typical AprilTag camera) | As `widthPixels`. |
| `diagonalFieldOfView` | characterized | 80° | guess (typical) | PhotonVision's lens calibration, at the resolution the robot runs, reports it. |
| `frameRate` | characterized | 30 per second | guess (conservative) | [Camera latency](characterization.md#camera-latency-and-time-sync): results per second, on the robot's coprocessor and network. |
| `latency` | characterized | 30 ms | guess (conservative) | [Camera latency](characterization.md#camera-latency-and-time-sync): from the image's capture to its result reaching the robot. |

## Localization

### `VisionConfig`

`frc.robot.lib.localization.VisionConfig`: which camera results are thrown out, and how far the rest
are trusted. Per robot (`RobotConfig.vision`); every robot starts from `VisionConfig.DEFAULTS`. Five
are tunable live. Each image's outcome, with the number that decided it, is in
`Vision/<camera>/Outcomes`; the estimator's verdict in `Vision/<camera>/Decisions`.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `maxAmbiguity` | chosen | 0.2 | 2026 code (254 used 0.19) | Why: a single tag above it fits two poses about equally well, and the wrong one is far off. Change if "ambiguous single tag" throws out results that agreed with the estimate, or single tags under it are then rejected by the gate. |
| `maxHeightErrorMeters` | chosen | 0.2 m | 2026 code | Why: the robot is on the floor, and tags are where the field says; a solve that misses either by 20 cm is a misread. Check the "off the floor" and "height is off" outcomes against the other cameras. |
| `maxDistanceMeters` | chosen | 6 m | guess | Why: noise grows with distance squared, and beyond some distance a result adds more doubt than it removes. Set it from [camera noise](characterization.md#camera-noise): where a tag's noise passes what odometry drifts in a second or two. Check the "too far" outcomes. |
| `maxYawRateRadiansPerSecond` | chosen | 5 rad/s | top team (254: 5 rad/s over the last 0.3 s) | Why: motion blur, rolling shutter, and a capture time a little off all grow with turning. Check: images just under it should pass the gate as often as the robot's still ones. |
| `linearStdDevBaseline` | characterized | 0.02 m | 2026 code | [Camera noise](characterization.md#camera-noise): position error for one tag at 1 m. Check: in clean driving, the gate should reject about 1 in 1000 measurements; far more and this is too small. |
| `angularStdDevBaseline` | characterized | 0.03 rad | 2026 code | [Camera noise](characterization.md#camera-noise): heading error for one tag at 1 m. |
| `distanceExponent` | characterized | 2 | 2026 code (teams used 2 to 4) | [Camera noise](characterization.md#camera-noise): how fast the error grows with distance. |
| `headingConstrainedFactor` | characterized | 0.5 | top team (6328's factor for the same idea) | [Camera noise](characterization.md#camera-noise): how much less a position solved with the gyro's heading scatters. |
| `speedStdDevFactor` | characterized | 0.25 per m/s | unknown (the template credits top teams, none by name) | [Camera noise, while moving](characterization.md#while-moving). |
| `yawRateStdDevFactor` | characterized | 0.5 per rad/s | unknown (the template credits top teams, none by name) | [Camera noise, while moving](characterization.md#while-moving). |
| `maxHeadingStdDevToConstrain` | chosen | 5° (0.087 rad) | unknown | Why: a heading-constrained solve is only as good as the heading it's given; until the estimate knows its heading this well (a robot not yet placed), full solves find it. Check how soon after placing the robot its outcomes say "heading-constrained". |
| `minTagsForHeading` | chosen | 2 | guess (the fewest a multi-tag solve has) | Why: one tag's heading is too noisy and too ambiguous to correct the gyro. Raise it if two-tag solves pull the heading around (`Localization/Pose`'s heading stepping against `Drive/Gyro/Yaw`). |
| `maxYawRateForHeading` | chosen | 0.5 rad/s | unknown | Why: correct the gyro's heading only when the robot is turning slowly, where a capture time a little off barely moves the heading. |
| `maxReprojectionErrorForHeadingPixels` | chosen | 1 px | unknown | Why: only a solve that fits its tags well corrects the gyro. Check against [camera noise](characterization.md#camera-noise): a still camera's multi-tag solves should fit well under it. |

### `EstimatorConfig`

`frc.robot.lib.localization.EstimatorConfig`: how the pose estimator weighs odometry, the gyro, and
the cameras. Per robot (`RobotConfig.estimator`); every robot starts from
`EstimatorConfig.DEFAULTS`, which the code calls starting points, not measurements.

The estimator's uncertainty should match its error. In the simulator, compare
`Localization/PositionStdDevMeters` with `Sim/PoseErrorMeters`; on a robot, with how far the
cameras move the estimate when they see tags again after a stretch without. An estimate often off by
three times its uncertainty is too sure of itself, and its gate rejects good measurements; one never
off by half of it is too unsure, and the cameras pull it around.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `translationStdDevPerMeter` | characterized | 0.03 m | guess | [Odometry error](characterization.md#odometry-error), per drivetrain and tread. |
| `translationStdDevPerRadian` | characterized | 0.02 m | guess | [Odometry error](characterization.md#odometry-error): spinning in place. |
| `residualScale` | chosen | 1 | guess (all of it) | Why: wheels that disagree with each other put the robot's travel in doubt by about as much. Lower it if the uncertainty balloons in clean driving (`Localization/ResidualMeters` high, the cameras finding little to correct). |
| `tractionLimitedSlipFraction` | chosen | 0.25 | guess | Why: wheels at the edge of their grip slip some of the time. Check how far the cameras correct the estimate after stretches of `Localization/TractionLimited`. |
| `tiltStartDegrees` | chosen | 5° | guess | Why: a small tilt is carpet and bumpers, not wheels off the floor. Check: `Localization/TiltDegrees` stays under it in flat driving. |
| `tiltZeroDegrees` | chosen | 25° | top team (6328) | Why: by this tilt, some wheels are off the floor. Check with a [terrain crossing](characterization.md#terrain-crossings). |
| `slipCorrelationSeconds` | chosen | 0.5 s | guess | Why: a slip (a push, a launch) keeps its direction for a while, so its errors add up rather than average out. |
| `collisionStdDevMeters` | chosen | 0.25 m | guess | Why: a hit moves the robot further than its wheels see. Check: after logged hits (`Localization/Collision`), how far the cameras moved the estimate; it should cover the largest. |
| `collisionWindowSeconds` | chosen | 0.25 s | guess | Why: one hit jolts for several loops; count it once. Check: one hit's jolts span less than it. |
| `handlingStdDevPerSqrtSecond` | chosen | 0.02 m | guess (the idea: 254's loose trust while disabled) | Why: a disabled robot gets carried, and its wheels can't tell, so the cameras should lead. Check: placed and disabled, the estimate settles on the cameras before the match. |
| `headingDriftDegreesPerMinute` | characterized | 0.5° | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise): the gyro's worst drift, standing still. |
| `headingStdDevDegreesPerTurn` | characterized | 0.5° | guess | [The gyro's scale, repeated](characterization.md#repeatability-of-the-gyros-scale): how far its scale may still be off after calibrating it. |
| `wheelHeadingStdDevDegreesPerTurn` | characterized | 10° | guess | [Odometry error](characterization.md#odometry-error): heading from the wheels against the gyro's, spinning at driving speed. Used only while the gyro is lost. |
| `historySeconds` | chosen | 1.5 s | guess (254 kept 1 s) | Why: a camera measurement must arrive within it to be applied; several times any camera's [latency](characterization.md#camera-latency-and-time-sync). Check for "too late" in `Decisions`. |
| `futureToleranceSeconds` | chosen | 0.05 s | guess | Why: clocks differ a little, but a measurement stamped this far past the newest odometry has a wrong capture time. Check for "from the future" in `Decisions` ([time sync](characterization.md#camera-latency-and-time-sync)). |
| `visionTimeoutSeconds` | chosen | 1 s | guess | Why: after a second without an accepted measurement, the estimate is dead reckoning, and says so. Check: `Localization/Mode` reads DEAD_RECKONING only where the cameras are really blind. |
| `disagreementWindowSeconds` | chosen | 0.5 s | guess | Why: how far back "lately" reaches for `RecentDisagreementMeters`, which the scoring gate reads. |
| `gateChiSquaredPosition` | chosen | 13.816 | math (chi-squared, 2 degrees of freedom: 99.9% of good measurements pass) | Why: rejects measurements its noise model says are almost certainly wrong. A good noise model rejects about 1 in 1000 good measurements; far more "gate:" rejections in clean driving mean the uncertainties are too small, not that the gate is too tight. |
| `gateChiSquaredPose` | chosen | 16.266 | math (3 degrees of freedom, 99.9%) | As `gateChiSquaredPosition`, for a measurement with heading. |
| `gateChiSquaredAxis` | chosen | 10.828 | math (1 degree of freedom, 99.9%) | As `gateChiSquaredPosition`, for a landmark. |
| `recoveryMeasurements` | chosen | 3 | guess | Why: three rejected measurements that agree with each other outvote odometry; with fewer, one bad camera could. Check: `Localization/Recoveries` counts only real knocks (a hit, a push, a bad placement). |
| `recoveryWindowSeconds` | chosen | 1 s | guess | Why: those measurements must be recent, so they describe the same moment. As `recoveryMeasurements`. |
| `recoveryAgreementMeters` | chosen | 0.25 m | guess | Why: measurements this close agree, given camera noise at range. Keep it above the noise at the distances the cameras see tags. |
| `recoveryStdDevMeters` | chosen | 0.25 m | guess | Why: after a jump, the estimate is that unsure, so the next measurements settle it. |
| `recoveryAgreementDegrees` | chosen | 5° | guess | Why: rejected measurements whose headings agree within this, and not with the gyro, mean the heading is wrong (a bad reset, a robot placed backwards). |
| `recoveryHeadingStdDevDegrees` | chosen | 2° | guess | Why: as `recoveryStdDevMeters`, for heading. |
| `lostAfterRejections` | chosen | 2 | guess | Why: two rejections in a row with none accepted, and the estimate says it's lost. Check: LOST in `Localization/Mode` away from real knocks means it's too low, or the noise model too sure. |
| `lostPositionStdDevMeters` | chosen | 1 m | guess | Why: after a heading recovery, the position was worked out with the wrong heading, so it may be far off. |

### `LocalizationGate`

`frc.robot.lib.localization.LocalizationGate`: whether an estimate is good enough to score from;
the robot uses `LocalizationGate.SCORING`. Check `Localization/ReadyToScore` and `NotReadyBecause`
in a match log: ready whenever the robot is lined up with tags in view, and never while the estimate
is off.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `maxVisionAgeSeconds` | chosen | 0.5 s | top team (254) | Why: odometry drifts after the cameras last checked it. |
| `maxDisagreementMeters` | chosen | 0.2 m | top team (254) | Why: cameras that just disagreed by more haven't settled the estimate. Lower it toward the task's tolerance once [camera noise](characterization.md#camera-noise) says the cameras do better. |
| `maxPositionStdDevMeters` | chosen | 0.15 m | unknown | Why: the scoring task's tolerance. Set it from the game: how far off a shot or a placement may land. |
| `maxTiltDegrees` | chosen | 5° | unknown (the idea: 1678 and 6328 wouldn't shoot tilted) | Why: a tilted robot aims off, and its cameras solve worse. |

### `ImuCrossCheck.Settings`

`frc.robot.lib.localization.ImuCrossCheck.Settings`: how closely the gyro and the backup IMU must
agree before an alert says one is failing; the robot uses `Settings.DEFAULTS`. Check
`Localization/ImuDisagreementDegrees`: well inside the tolerance in normal driving, and over it only
for a real fault.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `windowSeconds` | chosen | 1 s | guess | Why: long enough to see two IMUs turn apart, short enough to catch a frozen one in a second. |
| `toleranceDegrees` | chosen | 3° | guess | Why: above what the two drift and scatter apart in the window ([gyro drift and noise](characterization.md#gyro-drift-and-noise), for both). |
| `scaleTolerance` | chosen | 0.05 | guess | Why: above the two IMUs' difference in scale; the gyro's is calibrated, the backup's isn't ([the backup's scale](characterization.md#repeatability-of-the-gyros-scale), S13). |

### `TerrainCrossing.Settings`

`frc.robot.lib.localization.TerrainCrossing.Settings`: what counts as climbing, level, and one
feature; the robot uses `Settings.DEFAULTS`. Check them with a
[terrain crossing](characterization.md#terrain-crossings) (M7), and set them again at kickoff from
the new field's ramps.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `steepDegrees` | chosen | 10° | top team (1678) | Why: under the ramps' slope (2026's: 15°), over anything flat driving reads. |
| `levelDegrees` | chosen | 5° | top team (1678) | Why: what a flat floor reads, bumpers and carpet included. |
| `levelSeconds` | chosen | 0.1 s | unknown | Why: five loops level is off the feature, not a wobble on its top. |
| `topSeconds` | chosen | 1.5 s | guess | Why: time to cross a feature's flat top; longer is two features, or a stop on top. |

## The simulator

### `SensorErrors`

`frc.robot.sim.SensorErrors`: how the simulated robot's sensors differ from the truth.
`SensorErrors.TYPICAL` is a robot partway through a season (the simulator and its scenarios use it);
`NONE` is exact, for tests of exact numbers. The code calls TYPICAL's values plausible estimates:
replace them with a real robot's once it's calibrated.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `wheelRadiusScale` | characterized | 0.985, 0.99, 0.98, 0.988 | guess | The spread of each wheel's radius in [the wheel radius](calibration.md#the-wheel-radius)'s report, partway through a season, over the radius configured. |
| `steerOffsetDegrees` | characterized | 0.5, -0.3, 0, 0.4 | guess | How far modules point off straight after their offsets are set: square the wheels against a straightedge, and read each module's angle (`Drive/Module<name>/SteerAngle`). |
| `cameraMountErrors` | characterized | none | guess (calibrated mounts stay put) | How far a mount moves between calibrations: [a camera's mount](calibration.md#a-cameras-mount)'s "before" residual, run again weeks later. Scenarios set their own. |
| `seed` | chosen | 1 | none | Any number: fixed, so a run repeats exactly. |

### `SensorErrors.GyroErrors`

One gyro's errors. TYPICAL gives the gyro and the backup IMU each their own; EXACT has none.

| Value | Kind | Today | Source | How to measure it, or why |
|---|---|---|---|---|
| `scale` | characterized | 1.003 (backup 1.005) | guess | [The gyro's scale](calibration.md#the-gyros-scale) before it's calibrated (one over its first `gyroYawScale`), and [the backup's](characterization.md#repeatability-of-the-gyros-scale) (S13). |
| `driftDegreesPerMinute` | characterized | 0.2 (backup 1.0) | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise), a typical run (S13 for the backup). |
| `noiseDegrees` | characterized | 0.02 (backup 0.05) | guess | [Gyro drift and noise](characterization.md#gyro-drift-and-noise), a typical run. |

## Tuning limits

How far the pit may take each value a mechanism tunes live, declared with the value in the
mechanism's `tuning`. All are chosen: wide enough to tune in, narrow enough that a slip of a finger
can't hurt the robot. The record's own checks still apply inside them. Change a limit when tuning on
a robot keeps wanting to go past it, after asking why.

### `drivetrain` tuning

`Drive.tuning`.

| Knob | Limits | Source | Why |
|---|---|---|---|
| `gains.driveKs` | 0 to 1 V | guess | Static friction takes a few tenths of a volt; past a volt, the robot creeps. |
| `gains.driveKv` | 0 to 5 V per m/s | guess | Twice the ideal 2.54: room for a real drivetrain's losses, not for a unit mistake. |
| `gains.driveKp` | 0 to 20 | unknown | Ten times today's: room to find where it buzzes. |
| `gains.steerKp` | 0 to 100 | unknown | About six times today's. |
| `gains.steerKd` | 0 to 5 | unknown | Far above today's; damping this high only adds noise. |
| `gains.skewLookahead` | 0 to 0.1 s | unknown | Five loops; past it, the robot steers ahead of itself. |
| `limits.driveStatorCurrent` | 10 to 120 A | unknown | Up to Phoenix 6's own default stator limit (120 A); under 10 A the robot barely moves. |
| `limits.driveSupplyCurrent` | 10 to 80 A | unknown | Room above today's 60 A; four motors drawing more brown out a battery. |
| `limits.steerStatorCurrent` | 5 to 80 A | unknown | Steering needs little; the top is for finding out how little. |
| `limits.maxLinearSpeed` | 0.5 to 6 m/s | unknown | Slow enough for a first drive; the record refuses anything past the free speed. |
| `limits.maxAngularSpeed` | 0.1 to 3 rotations/s | unknown | Slow enough for a first drive, and three times today's. |

### `vision` tuning

`Vision.tuning`.

| Knob | Limits | Source | Why |
|---|---|---|---|
| `maxAmbiguity` | 0 to 1 | math (an ambiguity is a ratio from 0 to 1) | Its whole range. |
| `maxHeightErrorMeters` | 0 to 1 m | unknown | A meter off the floor is no robot. |
| `linearStdDevBaseline` | 0.001 to 1 m | unknown | Never zero, which would trust a camera completely. |
| `angularStdDevBaseline` | 0.001 to 1 rad | unknown | As `linearStdDevBaseline`. |
| `distanceExponent` | 0 to 4 | top team (teams used 2 to 4) | The range teams used, and flat. |

## Numbers still written in code

These aren't configuration yet, and the test doesn't check this list. A number worth measuring
belongs in a configuration record, where the test makes the register cover it; until it moves, it's
listed here. Other numbers in code (deadbands, the calibration procedures' own settings) are the
code's choices, explained beside them.

| Where | Number | Kind | Source | How to measure it, or why |
|---|---|---|---|---|
| `VisionSim`: the farthest tag a simulated camera detects | 7 m | characterized | unknown | [Camera noise](characterization.md#camera-noise): the farthest a tag is found in most images. |
| `VisionSim`: tag corner noise | 0.25 px average, 0.08 px spread | characterized | datasheet (PhotonVision's simulation example) | [Camera noise](characterization.md#camera-noise): how far the corners scatter, standing still. |
| `VisionSim`: latency's spread | 5 ms | characterized | datasheet (PhotonVision's simulation example) | [Camera latency](characterization.md#camera-latency-and-time-sync). |
| `ModuleIOSim`: drive and steering inertia | 0.025 and 0.004 kg m² | characterized | datasheet (Phoenix Tuner X's simulation defaults, as AdvantageKit's template uses) | [Feedforward](characterization.md#feedforward): SysId on one module (M4). |
| `Localization`: a jolt that's a collision | 20 m/s² | chosen | top team (6800) | Why: hard driving stays near 1 g. Check: the IMU's acceleration (`Drive/Gyro/AccelerationXMetersPerSecSq`, `AccelerationYMetersPerSecSq`) stays well under it in hard driving, and goes over it in hits. |
| `Localization`: a tilt past a bump's slope still counted as the ground | 3° | calibrated | guess | [Terrain crossings](characterization.md#terrain-crossings) (M7): bumpers, suspension, and the IMU's mount. |
| `Localization`: how closely a crest places the robot | 5 cm | calibrated | guess | [Terrain crossings](characterization.md#terrain-crossings) (M7). |
| `Drive`: a wheel this far short of its speed is at its grip's edge | 0.5 m/s | chosen | unknown (the idea: 581 and 1678) | Why: a wheel that can't keep up is slipping or stalled. Check: `Localization/TractionLimited` turns on in pushing matches, and not in clean driving. |
| `Vision`: how far back "lately" reaches for turning | 0.3 s | chosen | top team (254) | Why: an image taken just after a fast turn is still blurred, and its capture time matters most. |

## Records that aren't configuration

| Record | Why it isn't configuration |
|---|---|
| `ImuReadings` | What one IMU read in one loop. |
| `DrivetrainReadings` | What the drivetrain read in one loop, for localization. |
| `RobotStatus` | What the drive team's dashboard shows, built each loop. |
| `DriveHardware` | The drivetrain's devices, handed to it: hardware, not numbers. |
| `Scenario` | A named situation for the simulator. Its numbers are what a test needs, written in `Scenarios`. |
