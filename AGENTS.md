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
| `CommandXboxController` | `org.wpilib.command3.button.CommandGamepad`, built with the scheduler |
| `ChassisSpeeds`, `SwerveModuleState` | `ChassisVelocities`, `SwerveModuleVelocity` |
| `SmartDashboard.putX` | `Telemetry.log(name, value)`, and AdvantageKit's `Logger.recordOutput` |
| `Timer.getFPGATimestamp()` seconds | `RobotController.getTime()` nanoseconds; the DataLog API takes nanoseconds too |
| A mechanism class per arm or roller, CAN IDs in a constants file | Describe each joint in one block in `Robots.java`: its shape, places, motors and their CAN IDs, its own sensors and its control. The template builds its mechanism and its commands ([docs/mechanisms.md](docs/mechanisms.md)) |
| Constants classes | `@ConfigRecord` records, built by name through their generated builders, each value with a row in [docs/values.md](docs/values.md) |

Hardware sits behind AdvantageKit's IO pattern: a `*IO` interface with its `@AutoLog` inputs, one
implementation per kind of hardware, and one for the simulator.

## Checking your work

```bash
./gradlew test -Pquick --tests '*Arm*'   # the tests near your change, fast
./gradlew ci                             # everything CI runs; green before you're done
./gradlew spotlessApply                  # fixes formatting
./gradlew run                            # the simulator, driven from Pitwall's Sim view
```

`ci` runs the formatter, Error Prone and NullAway, the architecture tests (`ArchitectureTest`), the
coverage floor, and the values register's test, which fails until every configuration value has its
row. When a check fails, its message says what to do: do that. Never suppress a warning, loosen an
architecture rule, or delete a test to get green; if a rule looks wrong, say so instead.

The simulator's tests (`*SimTest`) are deterministic. One that fails sometimes is a bug: find its
cause, don't rerun it.

## Working with students

This template is for students learning to program a robot. When you write code for one, explain
what each part does and why, so they can debug it when you're not there, and prefer the plain
version they could have written over the clever one.

## Where things are

- [README.md](README.md): the quick start, what the template does and why, and the repository's
  layout
- [docs/mechanisms.md](docs/mechanisms.md): describe a mechanism in one block, use its commands
- [docs/driving.md](docs/driving.md): the drivetrain's moves
- [docs/values.md](docs/values.md): every configuration value, what it means, and how to find yours
- [docs/testing.md](docs/testing.md): the tests, and how to write one against the simulator
- [docs/simulator.md](docs/simulator.md): the simulator, its faults and scenarios
- [docs/calibration.md](docs/calibration.md), [docs/characterization.md](docs/characterization.md):
  measuring the real robot
- [docs/editors.md](docs/editors.md), [docs/upgrading.md](docs/upgrading.md),
  [docs/kickoff.md](docs/kickoff.md): editors, WPILib upgrades, a new season's field
