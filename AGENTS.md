# Instructions for coding agents

A robot template for WPILib 2027 (Systemcore, Java 25), Commands v3, AdvantageKit and Phoenix 6.
Most FRC code you've seen is WPILib 2025 and Commands v2, and much of it is wrong here: check the
list below before writing anything from memory, and read the code next to what you're changing.

## What's different from older FRC code

| Older code | Here |
|---|---|
| `edu.wpi.first.*` | `org.wpilib.*`: `org.wpilib.math.geometry.Pose2d`, `org.wpilib.command3.Command` |
| `SubsystemBase`, `CommandScheduler.getInstance()` | Commands v3 (`org.wpilib.command3`): a `Mechanism` holds what a command requires; the one `Scheduler` is passed in, `Scheduler.getDefault()` only in `Robot` |
| `InstantCommand`, `SequentialCommandGroup`, `andThen` chains | `Command.requiring(mechanism).executing(co -> { ... }).named("...")`, its steps written in order with its `Coroutine`: `co.await(command)`, `co.waitUntil(condition)`, `co.fork(command)` |
| `CommandXboxController`, bindings in `RobotContainer` | Commands and bindings in `Controls`, the team's own file: the robot hands it its definition, the joints (with a gate on anything the model says, a sensor's reading included: `joints.gate(...)`), the drivetrain and both gamepads (`RecordingGamepad`, a v3 `CommandGamepad` that records each binding) |
| `ChassisSpeeds`, `SwerveModuleState` | `ChassisVelocities`, `SwerveModuleVelocity` |
| `SmartDashboard.putX` | `Telemetry.log(name, value)`, and AdvantageKit's `Logger.recordOutput` |
| `Timer.getFPGATimestamp()` seconds | `RobotController.getTime()` nanoseconds; the DataLog API takes nanoseconds too |
| A mechanism class per arm or roller, CAN IDs in a constants file | Describe each joint in one block in `Robots.java`: its shape, places, motors and their CAN IDs, its own sensors and its control. The template builds its mechanism and its commands ([runtime/docs/mechanisms.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/mechanisms.md)) |
| Constants classes | `@ConfigRecord` records, built by name through their generated builders, each value with a row in [runtime/docs/values.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/values.md) |

Two projects hold the code. `frc.robot` (`src/`) is the team's robot program: `Main`, `Robots`,
`Controls`, and the team's robots. The runtime (`com.michaelgrundvig.frc.runtime`,
github.com/mikegrundvig/frc-robot-runtime) is the robot program every robot shares, which no team
writes, with the harness a team tests with (its test fixtures). The runtime never names a team class: `Main` hands it which robot
this is, the season's field (from the model) and the controls (`RobotProgram.start`), and `ArchitectureTest` keeps it so.
Change a team's behavior in `frc.robot`; a change to the runtime is a change for every robot. The
runtime is a repository of its own (like the robot model, localization and Pitwall), published to
the FRC libraries' Maven repository and used here by its version, `frc-libs` in
`gradle/libs.versions.toml`: change it in its own clone, built into this program with
`-PlocalLibs=<folder>` while you work ([docs/upgrading.md](docs/upgrading.md#developing-the-libraries)),
and publish a release for this build to move to.

Hardware sits behind AdvantageKit's IO pattern: a `*IO` interface with its `@AutoLog` inputs, one
implementation per kind of hardware, and one for the simulator.

## Checking your work

```bash
./gradlew test -Pquick --tests '*Arm*'   # the tests near your change, fast
./gradlew ci                             # everything CI runs; green before you're done
./gradlew spotlessApply                  # fixes formatting
./gradlew run                            # the simulator, driven from Pitwall's Sim view
```

`ci` runs the formatter, Error Prone and NullAway, the architecture tests (`ArchitectureTest`, the
runtime's and the team's), the coverage floor, the check that the runtime's versions of WPILib and
the vendor libraries are this build's, and, with `-PlocalLibs`, the libraries' own `ci`. The
runtime's has the values register's test, which fails until every configuration value has its row,
and the guides' examples' test (`DocExamplesTest`): every Java block in a runtime guide is code a
compiled source has. Write an example in the runtime's
`src/test/java/com/michaelgrundvig/frc/runtime/examples` and copy it out; a block that
needs code around it says `java fragment` on its fence. A Javadoc example is a region of compiled
code, `{@snippet class=com.michaelgrundvig.frc.runtime.ExampleRobot region=intake}`, marked in the
source with `// @start region=intake` and `// @end`; `ci` builds the Javadoc, which fails on one
that isn't there. Javadoc's `{@link}`s are checked as the code compiles. When a check fails,
its message says what to do: do that. Never suppress a warning or delete a test to get green.

When a check blocks the natural solution instead (a library already on the classpath, a JDK API, a
simpler design), stop that part of the work: don't write around it, and don't loosen the check.
Add an entry to [runtime/docs/rule-requests.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/rule-requests.md), and say so in your summary.

An architecture rule's `because` names, in one line, the bug it prevents. Before adding one, ask:
can Gradle's project boundaries say it? Can an existing check, such as Error Prone, say it? Can it
deny the hazard, rather than allow a list?

The simulator's tests (`*SimTest`) are deterministic. One that fails sometimes is a bug: find its
cause, don't rerun it.

## Working with students

This template is for students learning to program a robot. When you write code for one, explain
what each part does and why, so they can debug it when you're not there, and prefer the plain
version they could have written over the clever one.

## Where things are

- [README.md](README.md): the quick start, what the template does and why, and the repository's
  layout
- [runtime/docs/mechanisms.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/mechanisms.md): describe a mechanism in one block, use its commands
- [runtime/docs/driving.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/driving.md): the drivetrain's moves
- [runtime/docs/autos.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/autos.md): declaring, choosing, running and checking autos
- [runtime/docs/values.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/values.md): every configuration value, what it means, and how to find yours
- [runtime/docs/testing.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/testing.md): the tests, and how to write one against the simulator
- [runtime/docs/simulator.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/simulator.md): the simulator, its faults and scenarios
- [runtime/docs/localizers.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/localizers.md): choosing the pose estimator (the
  default one, WPILib's, or your own), and the contract a localizer keeps
- [runtime/docs/calibration.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/calibration.md), [runtime/docs/characterization.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/characterization.md):
  measuring the real robot
- [docs/editors.md](docs/editors.md), [docs/upgrading.md](docs/upgrading.md),
  [docs/kickoff.md](docs/kickoff.md): editors, WPILib upgrades, a new season's field
