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
> drivetrain drives in the simulator; code for real motor controllers is still to come. Parts that
> only stand in until a library ships for 2027 are listed in [docs/stand-ins.md](docs/stand-ins.md).

## Quick start

You need git and any Java installation (to start the Gradle wrapper). Gradle downloads the exact
JDK the robot code needs; no WPILib installer is required to build, test, or simulate.

```bash
./gradlew ci                                 # every check CI runs; run before pushing
./gradlew test -Pquick --continuous          # rerun tests on every save while you work
./gradlew run                                # the simulator, enabled on a start and drivable from the keyboard
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
| **Architecture tests** ([ArchUnit](https://www.archunit.org)) | Structural rot: components that reach into robot code, static mutable state that makes behavior depend on what ran before, and configuration written by position, where two values can swap unnoticed. |
| **Tests** (JUnit 6, [AssertJ](https://assertj.github.io/doc/)) | Behavior regressions, including simulation tests that exercise physics. |
| **Coverage floor** (JaCoCo) | Code with no tests at all. The build fails below 50% line or branch coverage (`coverage-floor.properties`): enough to catch untested code, low enough not to slow heavy development. |
| **Formatting** (google-java-format via Spotless) | Diff noise and review friction. Your local build formats automatically; CI only checks. |

Also built in:

- **One build, everywhere:** a Gradle toolchain pins Java 25 and downloads it when missing, and the
  version catalog (`gradle/libs.versions.toml`) holds every build and test version in one place.
- **Every log says which code produced it:** the robot records the git SHA, branch, build time, and
  whether anything was uncommitted (`gradle/build-info.gradle`).
- **Pitwall, the robot's own web page** (http://localhost:5820 in the simulator,
  http://robot.local:5820 on the robot), started with one line in `Robot.java`:
  - **Status** at a glance: which robot and code, the mode, the battery. Add your own with a line:
    `pitwall.status().add("Arm homed", arm::isHomed)`.
  - **Actions**: routines such as pit checks, run from the page with their state shown live, only
    while the robot is enabled in utility mode and never on the field:
    `pitwall.actions().add("Home arm", arm::home)`.
  - **Live tuning** of the values marked `@Tunable`, such as gains and current limits. Each tuned
    value is checked exactly like a committed one and locked out while the field is connected.
    Nothing is saved on the robot: the page lists each change as the builder call that commits it
    to `Robots.java`.
  - **Every WPILib tunable** in the program, through a tee around WPILib's own NetworkTables
    backend: dashboards such as Elastic see and change them as always (the configuration's
    `@Tunable` values too, under `Config/`), but a change applies only while tuning is on, never on
    the field, and is logged for replay.

  With AdvantageKit, every request from the page is logged, so replays reproduce it. Pitwall lives
  in `pitwall/`.
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
  applied at the moment their image was taken, however late they arrive; every measurement gated
  against the estimate, so a misread tag can't move it; and recovery when the cameras agree with
  each other that odometry is wrong (a collision). It reports what it rests on (tracking,
  dead reckoning, lost, gyro lost) and how sure it is, on Pitwall and in the log. Swerve and tank
  kinematics plug in; so will any other drivetrain.
- **A simulator that knows where the robot really is:** its wheels and gyro report the truth with
  the errors a season-worn robot has (worn tread, encoder zeros a little off, gyro scale, drift, and
  noise; `SensorErrors`), so odometry drifts as a real robot's does, and a wheel can be made to slip
  or the robot shoved. The field view shows the robot where it believes it is and a "Truth" where
  it is; AdvantageScope gets `Sim/PoseErrorMeters`. Localization is tested against that gap, not
  against a perfect world: on a 15-second drive, odometry alone ends 12 cm off, and with a camera
  the estimate stays within 3.4 cm.
- **Nothing reaches the robot unchecked:** `deploy` runs every check first, and refuses to run
  without a team number.
- **Dependency updates** for build tools and CI actions arrive as weekly pull requests (Dependabot).
  Robot libraries (WPILib, vendordeps) are updated deliberately, per WPILib release.

## Layout

| Path | What |
|---|---|
| `src/main/java/frc/robot/` | The robot program. |
| `src/main/java/frc/robot/Robots.java` | Your robots: size, mass, drivetrain, and cameras, one typed configuration each, every value set by name and checked when built. |
| `src/main/java/frc/robot/Rebuilt.java` | The field the robot plays on: 2026's, until kickoff. Its starts, targets, and zones, written for the blue alliance. |
| `src/main/java/frc/robot/RobotContainer.java` | Builds each mechanism with the hardware for the mode (real, simulated, or replayed from a log) and binds the controls to commands. |
| `src/main/java/frc/robot/subsystems/drive/` | The swerve drivetrain, in AdvantageKit's IO pattern: the drive logic, plus one IO class per kind of hardware. Simulated today; add one for your motor controllers. A [stand-in](docs/stand-ins.md) until AdvantageKit's 2027 swerve template ships. |
| `src/main/java/frc/robot/lib/` | The component library: reusable, fully unit-tested parts that never depend on robot code (enforced), such as the typed configuration (`lib/config`), the field model (`lib/field`, which also writes down the field and robot frames), and localization's measurement types (`lib/localization`). |
| `src/test/java/` | Tests, including the architecture rules (`ArchitectureTest`) and simulation tests that drive the robot (`RobotContainerSimTest`, `subsystems/drive`). |
| `pitwall/` | Pitwall, the robot program's own web page and API: status, actions, and live tuning. Its own Gradle project and package (`com.michaelgrundvig.frc.pitwall`), free of robot code, so it can become a library teams pull in. `Robot.java` starts it. |
| `sim/` | Default simulator GUI settings, used on first `./gradlew run`. |
| `vendordeps/` | Vendor libraries, as in any WPILib project. |
| `docs/` | Guides, starting with [editors and commands](docs/editors.md). |
| `gradle/` | The build's supporting rules: `quality.gradle` (checks), `build-info.gradle` (build metadata), `simulation.gradle` (simulator defaults), and the version catalog. |

## License

[BSD 3-Clause](LICENSE). Parts of `Main.java`, `Robot.java`, `Constants.java`, and `ModuleIOSim.java` derive from
AdvantageKit's templates (BSD 3-Clause, Littleton Robotics); WPILib is BSD 3-Clause (FIRST and other
WPILib contributors). Their licenses are in [`licenses/`](licenses/).
