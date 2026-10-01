# Stand-ins

Parts of this template exist only because the real 2027 version hasn't shipped yet. They are
throwaway: when the real one arrives, delete ours and use it. Add a row when you add a stand-in, and
delete the row when you replace it.

| Stand-in | Stands in for | Replace when |
|---|---|---|
| The swerve drivetrain, `src/main/java/frc/robot/subsystems/drive/`, including its CTRE hardware code (`PhoenixDrivetrain`: TalonFX drive and steering, CANcoders, a Pigeon 2, and the thread that reads them at 250 Hz) | AdvantageKit's swerve templates, which bring the same for CTRE and REV hardware. Only the drivetrain goes: where the robot is belongs to `subsystems/localization` (the estimator, choosing between the gyro and the backup IMU, trusting the wheels, terrain and its landmarks), and the simulated world to `frc.robot.sim`, and both stay. Theirs needs one addition, an adapter that reports `DrivetrainReadings` each loop (every wheel reading, both IMUs, the velocity, and whether the wheels are at their grip's edge), and to take its pose from `Localization` rather than its own estimator. Keep what ours adds if theirs lacks it: the steer-drive coupling, the current-limit flag for slip, the IMU's attitude and acceleration. | AdvantageKit publishes its 2027 swerve templates, planned for WPILib's 2027 beta (AdvantageKit 27.0.0-alpha-6 release notes) |
| PhotonLib, pinned to a development build (`vendordeps/photonlib.json`, dev-v2027.0.0-alpha-2-69-g71416112) | A PhotonLib release for WPILib alpha 7. Its one 2027 release (alpha-2, May 2026) targets alpha 5, and GradleRIO alpha 7 refuses it. The coprocessor must run the same PhotonVision version as the pinned library. | PhotonVision releases a build for the WPILib version in use |

## Alpha dependencies

Released, but alpha: each is tied to one WPILib alpha, and moves with it.

- **Phoenix 6 26.70.0-alpha-2** (`vendordeps/Phoenix6-26.70.0-alpha-2.json`, from WPILib's
  vendor-json-repo, `2027_alpha7`): the version CTRE pairs with WPILib alpha 7. Every CTRE device
  needs firmware 26.70 to talk to it (Phoenix Tuner X Preview flashes it), and a CANivore on
  Systemcore needs CTRE's `canivore-usb` package (WPILib's SystemcoreTesting repository,
  CTR-Phoenix). No Pro license is assumed: no FOC, and the steering reads its CANcoder as a remote
  sensor.

## Waiting on

Not built here at all, because the library has no 2027 build yet. Add these when it does.

- **Autos:** PathPlannerLib. (ChoreoLib already has a 2027 alpha build.)
- **Physics simulation** (traction, collisions, game pieces): maple-sim. Until then, simulation tests
  the code, not the robot's traction or power. Power includes the drive motors' supply current
  limit: the simulator models neither it nor Phoenix's step-down from it
  (`SwerveConfig.Limits.driveSupplyLowerCurrent` and `driveSupplyLowerTime`), and its battery holds
  12 V, so the supply current a simulated module reports is what its motor would draw with no supply
  limit. At the template's 30 A stator limit that changes nothing: a drive motor draws under 30 A
  from the battery, below both limits. Raise the stator limit past them, and the simulator lets a
  motor draw more from the battery than its motor controller would. The simulator keeps the robot's true pose apart from
  its sensors (`DriveSimulation`, `SensorErrors`), the drive and steering motors obey their stator
  current limits, the steering's loop runs as a TalonFX's does (every millisecond, on the angle its
  CANcoder sends every 10 ms), the robot rides over the field's bumps (tilted and raised by the
  ground under its wheels), and faults can be made while it runs, from the Pitwall page, by name
  (`frc.robot.sim.Scenarios`, `./gradlew run -Pscenario=<name>`), or in a test
  (`frc.robot.sim.Faults`, listed in [simulator.md](simulator.md)): a slipping wheel, a push, a hit
  the IMU feels, a gyro unplugged, frozen, or reset, a camera knocked off its mount, frozen, or
  late, a module's encoders gone stale. Wheels still never slip on their own, and no one wheel can
  leave the floor: `wheel-lifted` tilts the robot by hand (which holds over any bump for the rest of
  the run) and scales every wheel's grip alike, so its wheels never disagree as a lifted one's
  would, and the robot isn't raised. The steering has no friction: a wheel's scrub on the carpet
  and its gearbox's drag need measuring on a real module first (`CurrentLimitedMotorSim` says where
  they go). And it hears its CANcoder at the rate the code asks for
  (`PhoenixDrivetrain.ENCODER_FREQUENCY_HZ`), which no robot has yet confirmed.

## Scenarios the simulator can't make yet

Faults a robot meets that the simulator can't make yet are listed in one place, [the simulator's
own guide](simulator.md#what-the-simulator-cant-make-fail-yet).
