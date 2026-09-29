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
./gradlew run                                # the simulator, enabled and drivable from the keyboard
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
| **Coverage ratchet** (JaCoCo) | New code without tests. Coverage may never drop below the floors in `coverage-floor.properties`; there is no percentage target to game. |
| **Formatting** (google-java-format via Spotless) | Diff noise and review friction. Your local build formats automatically; CI only checks. |

Also built in:

- **One build, everywhere:** a Gradle toolchain pins Java 25 and downloads it when missing, and the
  version catalog (`gradle/libs.versions.toml`) holds every build and test version in one place.
- **Every log says which code produced it:** the robot records the git SHA, branch, build time, and
  whether anything was uncommitted (`gradle/build-info.gradle`).
- **Live tuning from any browser:** the robot program serves its own web page, Pitwall
  (http://localhost:5800 in the simulator, http://robot.local:5800 on the robot), where the values
  marked `@Tunable`, such as gains and current limits, can be changed while the robot runs. Each
  change is checked exactly like a committed value, logged so replays reproduce it, and locked out
  while the field is connected. Nothing is saved on the robot: the page lists each change as the
  builder call that commits it to `Robots.java`. Pitwall lives in `pitwall/`.
- **Nothing reaches the robot unchecked:** `deploy` runs every check first, and refuses to run
  without a team number.
- **Dependency updates** for build tools and CI actions arrive as weekly pull requests (Dependabot).
  Robot libraries (WPILib, vendordeps) are updated deliberately, per WPILib release.

## Layout

| Path | What |
|---|---|
| `src/main/java/frc/robot/` | The robot program. |
| `src/main/java/frc/robot/Robots.java` | Your robots: size, mass, drivetrain, and cameras, one typed configuration each, every value set by name and checked when built. |
| `src/main/java/frc/robot/RobotContainer.java` | Builds each mechanism with the hardware for the mode (real, simulated, or replayed from a log) and binds the controls to commands. |
| `src/main/java/frc/robot/subsystems/drive/` | The swerve drivetrain, in AdvantageKit's IO pattern: the drive logic, plus one IO class per kind of hardware. Simulated today; add one for your motor controllers. A [stand-in](docs/stand-ins.md) until AdvantageKit's 2027 swerve template ships. |
| `src/main/java/frc/robot/lib/` | The component library: reusable, fully unit-tested parts that never depend on robot code (enforced), such as the typed configuration (`lib/config`). |
| `src/test/java/` | Tests, including the architecture rules (`ArchitectureTest`) and simulation tests that drive the robot (`RobotContainerSimTest`, `subsystems/drive`). |
| `pitwall/` | Pitwall, the robot program's own web page and API (live tuning so far): its own Gradle project and package (`com.michaelgrundvig.frc.pitwall`), free of robot code, so it can become a library teams pull in. `Robot.java` wires it in, and `TuningLog.java` logs its tuning requests with AdvantageKit. |
| `sim/` | Default simulator GUI settings, used on first `./gradlew run`. |
| `vendordeps/` | Vendor libraries, as in any WPILib project. |
| `docs/` | Guides, starting with [editors and commands](docs/editors.md). |
| `gradle/` | The build's supporting rules: `quality.gradle` (checks), `build-info.gradle` (build metadata), `simulation.gradle` (simulator defaults), and the version catalog. |

## License

[BSD 3-Clause](LICENSE). Parts of `Main.java`, `Robot.java`, `Constants.java`, and `ModuleIOSim.java` derive from
AdvantageKit's templates (BSD 3-Clause, Littleton Robotics); WPILib is BSD 3-Clause (FIRST and other
WPILib contributors). Their licenses are in [`licenses/`](licenses/).
