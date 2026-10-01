# FRC Robot Template

A starting point for FRC robot code that gets the fundamentals right from the first commit: typed
configuration, simulation you can test against, checks that catch real bugs before they reach the
robot, and a build that works the same on every laptop and in CI.

It builds on the standard tools rather than replacing them: [WPILib](https://docs.wpilib.org)
(2027, Systemcore), [AdvantageKit](https://docs.advantagekit.org) for logging and replay, and
WPILib's Commands v3. The layout is the standard WPILib project layout, so WPILib's tools and every
WPILib and AdvantageKit guide apply unchanged. Use it by clicking **Use this template**, then fill in
your robot.

> **Status:** early. It tracks WPILib's 2027 prereleases (currently 2027.0.0-alpha-7). The swerve
> drivetrain drives in the simulator, and has code for CTRE hardware (TalonFX, CANcoder, Pigeon 2)
> tested against CTRE's simulated devices, not yet on a robot. Parts that only stand in until a
> library ships for 2027 are listed in [docs/stand-ins.md](docs/stand-ins.md).

## Quick start

You need git and any Java installation (to start the Gradle wrapper). Gradle downloads the exact
JDK the robot code needs; no WPILib installer is required to build, test, or simulate.

```bash
./gradlew ci                                 # every check CI runs; run before pushing
./gradlew test -Pquick --continuous          # rerun tests on every save while you work
./gradlew run                                # the simulator, enabled on a start and drivable from the keyboard
./gradlew run -Pgui=false                    # the simulator without WPILib's GUI: Pitwall's page drives it
./gradlew run -Pscenario=collision           # the simulator in a scenario (./gradlew scenarios lists them)
AKIT_LOG_PATH=logs/<log>.wpilog ./gradlew run -Preplay   # replay a log through the current code
./gradlew setTeamNumber -PteamNumber=NNNN    # once, before your first deploy
./gradlew deploy                             # to the robot, after every check passes
```

Every job is a Gradle command, so any editor works, or none. [docs/editors.md](docs/editors.md)
covers IntelliJ IDEA and VS Code, and lists every command.

## The fundamentals, and why

Every rule below runs on every build, locally and in CI, and fails the build when broken. Each one
catches a kind of bug that robot code otherwise accumulates quietly until it bites at an event.

| Check | What it catches |
|---|---|
| **NullAway** with [JSpecify](https://jspecify.dev) annotations | A null reaching code that doesn't expect it: the classic mid-match crash. Nothing may be null unless it is declared `@Nullable`, and the compiler proves every use is checked. |
| **Error Prone** (error level) | Code that compiles but is certainly wrong: exceptions created and never thrown, bad equality, misused APIs. |
| **WPILib's compiler plugin** | FRC-specific mistakes, such as a Commands v3 loop that never yields and freezes the robot. |
| **Compiler warnings as errors** (a curated set) | Unchecked casts, raw types, switch fall-through, and other near-certain bugs. |
| **Architecture tests** ([ArchUnit](https://www.archunit.org)) | Structural rot: components that reach into robot code, hardware, or the clock (so a replay would decide differently); vendor libraries outside the IO layer; subsystems reaching into each other; static mutable state that makes behavior depend on what ran before; and configuration written by position, where two values can swap unnoticed. |
| **Tests** (JUnit 6, [AssertJ](https://assertj.github.io/doc/)) | Behavior regressions, including simulation tests that exercise physics. |
| **Coverage floor** (JaCoCo) | Code with no tests at all. The build fails below 50% line or branch coverage overall, or below 50% of the lines in any one file, so a file with no tests fails even while the rest carries the total (`coverage-floor.properties`). Enough to catch untested code, low enough not to slow heavy development. |
| **Formatting** (google-java-format via Spotless) | Diff noise and review friction. Your local build formats automatically; CI only checks. |

Also built in:

- **One build, everywhere:** a Gradle toolchain pins Java 25 and downloads it when missing. Build
  and test tools are versioned in one place, the version catalog (`gradle/libs.versions.toml`);
  robot libraries, as in any WPILib project, by GradleRIO's version (in the catalog) and
  `vendordeps/`.
- **Every log says which code produced it:** the robot records the git SHA, branch, build time, and
  whether anything was uncommitted (`gradle/build-info.gradle`).
- **Pitwall, the robot's own web page** (http://localhost:5820 in the simulator,
  http://robot.local:5820 on the robot), started with one line in `Robot.java`:
  - **Status** at a glance: which robot and code, the mode, the battery. Add your own with a line:
    `pitwall.status().add("Arm homed", arm::isHomed)`.
  - **Actions**: routines such as pit checks, run from the page with their state shown live, only
    while the robot is enabled in utility mode and never on the field:
    `pitwall.actions().add("Home arm", arm::home)`.
  - **Experiments**: tuning a mechanism by test motion. A mechanism describes itself as a *rig*:
    its gains (values of its `LiveConfig`), how to command it, how far it may go, and what to
    record (`Drive.steeringRig`). An experiment runs a test motion on it from the page, such as
    the steering's step to 90° and back (`pitwall.experiments().add(new Experiment(...))`), with an
    action's safety, and only while the page that started it stays in touch. Each run is recorded
    at the sensors' own rate (250 Hz for the steering) and scored: rise, overshoot, settling,
    steady-state error, and peak voltage and current. The page lays the runs side by side, the
    best of each score marked, with Restore to put a run's gains back and the builder calls that
    commit them. A gain changes only the way any value does, through the checks below.
  - **The simulator's controls** (in the simulator only; a robot has none): the Driver Station
    (enable, the mode, the alliance station, the field system, the match time), driving from the
    keyboard or a gamepad, pausing and stepping, Stop, and failing the simulated robot's parts
    while it runs, which WPILib's simulator GUI can't do: a camera unplugged or knocked, the gyro
    frozen, a module's encoder gone stale, the robot hit. See [docs/simulator.md](docs/simulator.md).
  - **Pushed, not polled:** the page follows the robot through one event stream that carries
    only what changed, so a page left open costs the robot almost nothing.
  - **Live tuning, on WPILib's own tunables.** Every tunable in the program goes through a tee
    around WPILib's NetworkTables backend: dashboards such as Elastic see and change them as always,
    but a change applies only while tuning is on, never on the field, and within the tunable's
    limits, and it is logged for replay.
    - **A mechanism's configuration** (gains, current limits, thresholds) is tuned as a
      `LiveConfig`, a WPILib complex tunable the mechanism declares:
      `LiveConfig.of("drivetrain", committed).number("gains.steerKp", Limits.between(0, 100))`.
      Each value carries its tuning limits, the range that's safe to dial in live, as WPILib
      properties any NetworkTables tool can read. Before a change applies, it is checked against
      its limits, then by the configuration's own checks, then by the robot's rules across
      mechanisms that read it (`RobotRules`: two arms that could collide), so a refused value
      never reaches the robot, and the page gives every reason (a dashboard's snaps back, and the
      page shows why). Several values change together, all or none, even across mechanisms. Nothing is saved on the robot: the page lists each
      change as the builder call that commits it to `Robots.java`. When the field connects, every
      configuration returns to committed (printed first, so nothing is lost): a match runs the
      committed configuration.
    - **A plain WPILib tunable** is a switch the drive team flips (a camera off, a hold time,
      within its limits: `TunableDouble.createConfig(0.0, Limits.between(0, 30).config())`). A
      switch keeps its setting into a match, with an alert naming each one that differs from
      startup. Text, arrays, and structs pass straight through, unlocked, so choosers keep working
      on the field.

  **Which one a new value is:** a value that describes the robot (a gain, a current limit, a
  threshold, an uncertainty) is configuration: a field of the robot's configuration record, set in
  `Robots.java`, and, to tune it live, declared in its mechanism's `LiveConfig` with its limits
  (see `Drive.tuning`). A plain tunable is only for a switch.

  **Where a check goes:** what a value may be at all goes in its record, with `Checks` (`steerKp`
  zero or more; `cgHeight` below the robot's height); how far it may be tuned live goes with its
  `Limits` in the mechanism's `tuning`; a check that reads two mechanisms goes in `RobotRules`.
  Each is checked against the committed configuration in the tests and at startup.

  With AdvantageKit, every request from the page is logged, so replays reproduce it. Pitwall lives
  in `pitwall/`. It has no login: anyone on the robot's network can use it, as they can the Driver
  Station and NetworkTables, so give the robot's Wi-Fi a strong password (or turn it off at events).
  In the simulator it serves only the laptop it runs on (`-Dpitwall.network=true` serves the
  network too, to open it on a phone), and it answers only requests addressed to the robot, so a
  web page open on a connected laptop can't use it.
- **A field model** (`lib/field`, filled in for 2026 in `Rebuilt.java`): the field's AprilTags and
  symmetry, and the start positions, targets, and zones robot code refers to by name. Everything is
  written once, for the blue alliance, and one tested transform gives red's; targets near tags are
  derived from each alliance's own tags, so a layout measured at an event corrects both. The robot
  waits on the start its Driver Station location picks (1, 2, 3: left, center, right), drives
  field-relative from its own alliance's end, checks its placement before the match, and has
  zone triggers that look ahead along its path. In the simulator, change the alliance station
  while disabled to start somewhere else. At kickoff, write the new game's the same way.
- **A pose estimator built to be trusted** (`lib/localization`): 250 Hz odometry, with the gyro
  owning heading and the wheels' disagreement (slip) lowering their trust; camera measurements
  applied at the moment their image was taken, even when they arrive late (up to 1.5 s, and
  within their camera's usual latency plus a margin, or the clocks are out of step and the image
  is left out); every measurement gated
  against the estimate, so a misread tag can't move it; and recovery when the cameras agree with
  each other that odometry is wrong (a collision). A camera knocked off its mount counts for less
  until it's recalibrated, and says so; a task lining up on a target can trust only that target's
  tags. It reports what it rests on (tracking, dead reckoning, lost, gyro lost) and how sure it
  is, on Pitwall and in the log. Swerve and tank kinematics plug in; so will any other drivetrain.
- **A simulator that knows where the robot really is:** its wheels and gyro report the truth with
  the errors a season-worn robot has (worn tread, encoder zeros a little off, gyro scale, drift, and
  noise; `SensorErrors`), so odometry drifts as a real robot's does, and a wheel can be made to slip
  or the robot shoved. The field view shows the robot where it believes it is and a "Truth" where
  it is; AdvantageScope gets `Sim/PoseErrorMeters`. Localization is tested against that gap, not
  against a perfect world: on a 15-second drive, odometry alone ends 12 cm off, and with a camera
  the estimate stays within 3.4 cm.
- **Named scenarios for the simulator:** a fault in the robot's sensors or something that happens
  to it, by name: `./gradlew run -Pscenario=camera-low` starts the simulator with a camera knocked
  4 in low, and `collision`, `gyro-unplugged`, or `bump-crossing` happen once the robot is enabled.
  Each is a named set of the simulated world's live faults (`frc.robot.sim.Faults`), so the Pitwall
  page applies one to a running simulation too, and undoes what lasts. Placing the robot again (a
  new alliance station while disabled) undoes a move. `./gradlew scenarios` lists them all, and a
  misspelled name stops before the simulator starts, with the list. The mechanisms and
  localization are never told: they find out from the sensors, as on a robot, and the log names
  the scenario (`SimScenario`) and the faults on (`Pitwall/Simulator/Faults`). Tests use the same
  catalog (`frc.robot.sim.Scenarios`, and `RobotSim`'s `build` and `start`); what the simulator
  can't make yet is in [docs/simulator.md](docs/simulator.md).
- **Calibration from the Pitwall page:** the gyro's scale, the wheel radius, whether the IMU reads
  level, and each camera's mount, each measured on the robot and reported as the number to commit
  and where it goes ([docs/calibration.md](docs/calibration.md)).
- **A drive-team dashboard** (`layouts/elastic-layout.json`, for Elastic): a pre-match tab and a
  match tab, checked against the code on every build, so no widget sits blank on the field.
- **Loop health:** every loop is timed section by section, and alerts say when it overruns (and
  where the time went), when logging falls behind, and when the log drive is missing or filling.
- **Nothing reaches the robot unchecked:** `deploy` runs every check first, and refuses to run
  without a team number.
- **Dependency updates** for build tools and CI actions arrive as weekly pull requests (Dependabot).
  Robot libraries (WPILib, vendordeps) are updated deliberately, per WPILib release.

## Layout

| Path | What |
|---|---|
| `src/main/java/frc/robot/` | The robot program. At the top: `Robot` (logging, Pitwall, and the loop), `RobotContainer` (builds every mechanism for the mode, real, simulated, or replayed from a log, and binds the controls), `Robots` and `RobotConfig` (below), and `LoopHealth`. |
| `src/main/java/frc/robot/Robots.java` | Your robots: size, mass, drivetrain, and cameras, one typed configuration each, every value set by name and checked when built. |
| `frc.robot.subsystems` | The mechanisms, in AdvantageKit's IO pattern (the logic, plus one IO class per kind of hardware): `drive`, the swerve drivetrain, simulated and CTRE's (a [stand-in](docs/stand-ins.md) until AdvantageKit's 2027 swerve template ships); `localization`, where the robot is; `vision`, the cameras. |
| `frc.robot.commands` | Commands that aren't one mechanism's: the calibration procedures. |
| `frc.robot.game` | The game: the field the robot plays on, 2026's (`Rebuilt`) until [kickoff](docs/kickoff.md), with its starts, targets, and zones written for the blue alliance. |
| `frc.robot.dashboard` | What the drive team's dashboard shows, and the log stream to it. |
| `frc.robot.sim` | The simulated world: the drivetrain's physics, sensors that err as a season-worn robot's do, what can fail while it runs (`Faults`, for Pitwall's simulator controls), and the named scenarios (`Scenarios`). Only the simulator builds it. |
| `frc.robot.lib` | The component library: reusable, fully unit-tested parts that never depend on robot code (enforced). The typed configuration (`config`), the field model and its frames (`field`), the pose estimator and everything that feeds it (`localization`, whose package documentation is a reading map), the calibration math (`calibration`), and loop timing (`timing`). |
| `src/test/java/` | Tests, in the same packages as what they test, from single classes to the whole robot in the simulator, plus the architecture rules (`ArchitectureTest`). See [docs/testing.md](docs/testing.md). |
| `src/studies/java/` | Simulation studies that answer design questions, in their own source set so none of their code runs on the robot: where to mount the cameras, and how many ([docs/camera-placement.md](docs/camera-placement.md), `./gradlew cameraStudy`). |
| `pitwall/` | Pitwall, the robot program's own web page and API: status, actions, and live tuning. Its own Gradle project and package (`com.michaelgrundvig.frc.pitwall`), free of robot code, so it can become a library teams pull in. `Robot.java` starts it. |
| `layouts/` | The drive team's dashboard layout, for Elastic. |
| `sim/` | Default simulator GUI settings, used on the first `./gradlew run` with the GUI. |
| `vendordeps/` | Vendor libraries, as in any WPILib project. |
| `docs/` | Guides: [editors and commands](docs/editors.md), [testing](docs/testing.md), [the simulator](docs/simulator.md), [calibration](docs/calibration.md), [kickoff](docs/kickoff.md), [upgrading WPILib](docs/upgrading.md), and the [stand-ins](docs/stand-ins.md). |
| `gradle/` | The build's supporting rules: `quality.gradle` (checks), `build-info.gradle` (build metadata), `simulation.gradle` (simulator defaults), and the version catalog. |

## License

[BSD 3-Clause](LICENSE). Parts of `Main.java`, `Robot.java`, `Constants.java`, and `ModuleIOSim.java` derive from
AdvantageKit's templates (BSD 3-Clause, Littleton Robotics); WPILib is BSD 3-Clause (FIRST and other
WPILib contributors). Their licenses are in [`licenses/`](licenses/).

The libraries the robot program uses come under their own licenses. Only one is in this
repository: uPlot, the plotting library Pitwall's page loads from the robot (no internet at an
event), whose license is in [`licenses/`](licenses/). Gradle downloads the rest, so sharing the
source shares none of them. A built robot program (the jar `deploy` sends) contains them all, so
share source, never built jars:

| Library | License |
|---|---|
| WPILib, including Commands v3 | BSD 3-Clause |
| AdvantageKit | BSD 3-Clause |
| PhotonLib | MIT |
| uPlot 1.6.32, in `pitwall/` | MIT |
| PhotonVision's targeting library, which PhotonLib uses | GPL-3.0, whose terms reach a built program you share |
| CTRE Phoenix 6 | CTRE's own license, which doesn't allow redistributing it |
