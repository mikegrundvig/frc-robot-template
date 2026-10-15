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
> library ships for 2027 are listed in [runtime/docs/stand-ins.md](runtime/docs/stand-ins.md).

## Quick start

You need git and any Java installation (to start the Gradle wrapper). Gradle downloads the exact
JDK the robot code needs; no WPILib installer is required to build, test, or simulate.

```bash
./gradlew ci                                 # every check CI runs; run before pushing
./gradlew test -Pquick --continuous          # rerun tests on every save while you work
./gradlew run                                # the simulator, driven from Pitwall's Sim view (http://localhost:5820/#sim)
./gradlew run -Pgui                          # with WPILib's simulator GUI too, enabled and drivable from its keyboard
./gradlew run -Pscenario=collision           # the simulator in a scenario (./gradlew scenarios lists them)
./gradlew run -Probot=beta                   # the simulator running another of your robots, by name (Robots.all())
AKIT_LOG_PATH=logs/<log>.wpilog ./gradlew run -Preplay   # replay a log through the current code
./gradlew setTeamNumber -PteamNumber=NNNN    # once, before your first deploy
./gradlew deploy                             # to the robot, after every check passes
```

Every job is a Gradle command, so any editor works, or none. [docs/editors.md](docs/editors.md)
covers IntelliJ IDEA and VS Code, and lists every command.

The runtime, the robot model, localization and Pitwall are git submodules, each a repository of its
own, built as part of this build. After cloning, `git submodule update --init` fetches them;
`--recursive` isn't needed, since the runtime's own copies of the model, localization and Pitwall
(and localization's own copy of the model) are used only when each is built on its own.

## The fundamentals, and why

Every rule below runs on every build, locally and in CI, and fails the build when broken. Each one
catches a kind of bug that robot code otherwise accumulates quietly until it bites at an event.

| Check | What it catches |
|---|---|
| **NullAway** with [JSpecify](https://jspecify.dev) annotations | A null reaching code that doesn't expect it: the classic mid-match crash. Nothing may be null unless it is declared `@Nullable`, and the compiler proves every use is checked. |
| **Error Prone** (error level) | Code that compiles but is certainly wrong: exceptions created and never thrown, bad equality, misused APIs, and a public static array anyone can change. |
| **WPILib's compiler plugin** | FRC-specific mistakes, such as a Commands v3 loop that never yields and freezes the robot. |
| **Compiler warnings as errors** (a curated set) | Unchecked casts, raw types, switch fall-through, and other near-certain bugs. |
| **Architecture tests** ([ArchUnit](https://www.archunit.org)) | What Gradle's boundaries between projects can't see: components that reach a device or robot code; components and libraries that read the clock, an unseeded draw, another thread, or shared state (so a replay would decide differently); vendor libraries outside the IO layer; subsystems reaching into each other; the alliance read before the Driver Station says it; static mutable state that makes behavior depend on what ran before; and configuration written by position, where two values can swap unnoticed. |
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
    `pitwall.actions().add("Home arm", arm::home)`. One that mustn't run while the robot can move,
    such as zeroing a mechanism by hand, runs only while it's disabled instead, and only until
    the field system first connects (`Actions.When.DISABLED`).
  - **Experiments**: tuning a mechanism by test motion. A mechanism describes itself as a *rig*:
    its gains (values of its `LiveConfig`), how to command it, how far it may go, and what to
    record (`Drive.steeringRig`). An experiment runs a test motion on it from the page, such as
    the steering's step to 90° and back (`pitwall.experiments().add(new Experiment(...))`), with an
    action's safety, and only while the page that started it stays in touch. Each run is recorded
    at the sensors' own rate (250 Hz for the steering) and scored: rise, overshoot, settling,
    steady-state error, and peak voltage and current. The page lays the runs side by side, the
    best of each score marked, with Restore to stage a run's gains again and the builder calls
    that commit them. A gain changes only the way any value does, through the checks below.
  - **Devices**: a card in Pit for each device the robot program asks after (the venue it plays
    on, say), saying whether it's there and how it is, and ending in what can be done about it: a
    file to download from it or send to it, or a choice to make. The robot program asks the
    device, on Pitwall's server threads, two downloads at once at most and nothing on the field;
    the browser never reaches a device itself:
    `pitwall.devices().add(Devices.Device.named("venue", "Field", card))`.
  - **The simulator's controls**, in the page's Sim view (in the simulator only; a robot has
    none), in place of WPILib's simulator GUI: the Driver Station (enable, the mode, the alliance
    station, the field system, the match time), driving from the keyboard or a gamepad, pausing
    and stepping, Stop, and failing the simulated robot's parts while it runs, which the GUI can't
    do: a camera unplugged or knocked, the gyro frozen, a module's encoder gone stale, the robot
    hit. Each is an API too. See [runtime/docs/simulator.md](runtime/docs/simulator.md).
  - **World**: the robot's world in 3D, from above or around, drawn from the code's own beliefs:
    the field and its tags, the robot where localization believes it is with its wheels and
    cameras as configured (a camera's wedge pointing the wrong way is a typo in `Robots.java`,
    found at a glance), how sure the estimate is, and each camera's sight lines to the tags it
    saw, solid where the estimator accepted the measurement and dashed where it refused it. A
    click on one says why, and offers the camera's calibration. In the simulator the truth
    stands beside the estimate as a dashed ghost. It follows the same time strip as the Robot
    view, so a line that came and went is found again in the last five minutes, and a legend
    beside it says what each mark means and switches each layer. Pitwall draws it from the
    robot's log (three.js, served from the robot), and only a page showing it is sent what
    moves; a still robot sends nothing. Any robot program can add its own scene:
    [pitwall/README.md](pitwall/README.md).
  - **Robot**: the robot's joints as `Robots.java` describes them, live, one card each, drawn by
    its kind: an arc for a pivot, between its stops, with its named places; a speedometer for a
    roller, up to its free speed; a bar for an elevator. On each, a solid needle where the joint
    is and a dashed one where it was told to go, and under it who owns it, how its last move
    ended, and what holds it. An interlock shows on both joints it is about: with the roller
    waiting for the pivot to pass 110°, that 110° stands out on the pivot's card. Beside the
    joints: the sensors, the gates with what each waits for, the gamepads, and the bindings that
    just fired. It is drawn from the program's own log, the last five minutes of which Pitwall
    remembers, and a time strip along its bottom goes back through them: scrub, or go loop by
    loop, and the cards show the robot as it was, with what changed in that loop marked. Every
    loop in which something happened is marked on the strip and listed beside it in order, in
    words ("roller held: roller runs only when pivot at least 110°", then "roller no longer
    held"), a move that timed out or was refused standing out as a problem: an ordering mistake
    in an auto is read there. Scrubbing moves what the page shows, never the robot. See
    [pitwall/README.md](pitwall/README.md#the-robot-view).
  - **Pushed, not polled:** the page follows the robot through one event stream that carries
    only what changed, so a page left open costs the robot almost nothing.
  - **The page** has a view for each job, Pit, Tune, Experiment, and Changes (and Sim, in the
    simulator), under a state bar that always says which robot (SIM or ROBOT), whether it's
    heard, its mode, whether tuning is on, and what isn't committed. Edits are staged and applied
    together; a refusal's reasons show beside the values they concern, an edit outside its limits
    is caught before it's sent, and a big step (over a quarter of a value's range) is warned of.
    Nothing is lost silently: the session's changes and runs are kept in the tab, each
    restorable. Plain JavaScript modules in the jar (`pitwall/src/main/resources/.../page/`),
    with nothing to build.
  - **Live tuning, on WPILib's own tunables.** Every tunable in the program goes through a tee
    around WPILib's NetworkTables backend: dashboards such as Elastic see and change them as always,
    but a change applies only while tuning is on, never on the field, and within the tunable's
    limits, and it is logged for replay.
    - **A mechanism's configuration** (gains, current limits, thresholds) is tuned as a
      `LiveConfig`, a WPILib complex tunable the mechanism declares:
      `LiveConfig.of("drivetrain", committed).number("gains.steerKp", Limits.between(0, 100))`.
      Each value carries its tuning limits, the range that's safe to dial in live, as WPILib
      properties any NetworkTables tool can read. Before a change applies, it is checked against
      its limits, then by the configuration's own checks, so a refused value never reaches the
      robot, and the page shows every reason beside the value it concerns (a dashboard's snaps
      back, and the page shows why). Several values change together, all or none, even across
      mechanisms. Nothing is saved on the robot: the page lists each
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
  zero or more; `bumperTop` no higher than the robot); how far it may be tuned live goes with its
  `Limits` in the mechanism's `tuning`, which may come from what the robot is (the drivetrain's
  top speed is tuned up to its free speed). Both are checked against the committed configuration
  in the tests and at startup.

  **What a value means is its `@param`:** the page shows a configuration record's Javadoc
  beside its values, each value's `@param` as what it means (`@param steerKp volts per radian of
  error`), and dashboards see it as the value's `description` property. So write what the value
  does and in what unit, not its name again. Every tunable value needs one: `RobotTuningTest`
  fails without it, naming the record whose Javadoc to write. The build copies the Javadoc into
  the program as it compiles (the documentation processor, in the model's `docs-processor`), so
  the robot needs no source to show it.

  With AdvantageKit, every request from the page is logged, so replays reproduce it. Pitwall lives
  in `pitwall/`. It has no login: anyone on the robot's network can use it, as they can the Driver
  Station and NetworkTables, so give the robot's Wi-Fi a strong password (or turn it off at events).
  In the simulator it serves only the laptop it runs on (`-Dpitwall.network=true` serves the
  network too, to open it on a phone), and it answers only requests addressed to the robot, so a
  web page open on a connected laptop can't use it.
- **A field model** (in the robot model, `model/`, with each season's field in its `game` package: 2026's is `Rebuilt`): the field's AprilTags
  and symmetry, and the start positions, places, and zones robot code refers to by name. Everything
  is written once, for the blue alliance, and one tested transform gives red's; places near tags
  are derived from each alliance's own tags, so a layout measured at an event corrects both. The
  robot waits on the start its Driver Station location picks (1, 2, 3: left, center, right), drives
  field-relative from its own alliance's end, checks its placement before the match, and has zone
  triggers that look ahead along its path. In the simulator, change the alliance station while
  disabled to start somewhere else. At kickoff, write the new game's the same way.
- **A pose estimator built to be trusted** (`localization/`): 250 Hz odometry, with the gyro
  owning heading and the wheels' disagreement (slip) lowering their trust; camera measurements
  applied at the moment their image was taken, even when they arrive late (up to 1.5 s, and
  within their camera's usual latency plus a margin, or the clocks are out of step and the image
  is left out); every measurement gated
  against the estimate, so a misread tag can't move it; and recovery when independent sources
  agree that odometry is wrong (two cameras, two tags, or a camera and an IMU that felt a
  collision). A camera knocked off its mount counts for less
  until it's recalibrated, and says so; a task lining up on a target can trust only that target's
  tags. It reports what it rests on (tracking, dead reckoning, lost, gyro lost) and how sure it
  is, on Pitwall and in the log. The kinematics sit behind one interface. Swerve is the one
  written; another drivetrain's would plug in there.
- **A simulator that knows where the robot really is:** its wheels and gyro report the truth with
  the errors a season-worn robot has (worn tread, encoder zeros a little off, gyro scale, drift, and
  noise; `SensorErrors`), so odometry drifts as a real robot's does, and a wheel can be made to slip
  or the robot shoved. The field view shows the robot where it believes it is and a "Truth" where
  it is, and so does Pitwall's World view, as a dashed ghost; AdvantageScope gets
  `Sim/PoseErrorMeters`. Localization is tested against that gap, not
  against a perfect world: on a 15-second drive, odometry alone ends 12 cm off, and with a camera
  the estimate stays within 3.4 cm.
- **Named scenarios for the simulator:** a fault in the robot's sensors or something that happens
  to it, by name: `./gradlew run -Pscenario=camera-low` starts the simulator with a camera knocked
  4 in low, and `collision`, `gyro-unplugged`, or `bump-crossing` happen once the robot is enabled.
  Each is a named set of the simulated world's live faults (`com.michaelgrundvig.frc.runtime.sim.Faults`), so the Pitwall
  page applies one to a running simulation too, and undoes what lasts. Placing the robot again (a
  new alliance station while disabled) undoes a move. `./gradlew scenarios` lists them all, and a
  misspelled name stops before the simulator starts, with the list. The mechanisms and
  localization are never told: they find out from the sensors, as on a robot, and the log names
  the scenario (`SimScenario`) and the faults on (`Pitwall/Simulator/Faults`). Tests use the same
  catalog (`com.michaelgrundvig.frc.runtime.sim.Scenarios`, and `RobotSim`'s `build` and `start`); what the simulator
  can't make yet is in [runtime/docs/simulator.md](runtime/docs/simulator.md).
- **Calibration from the Pitwall page:** the gyro's scale, the wheel radius, whether the IMU reads
  level, and each camera's mount, each measured on the robot and reported as the number to commit
  and where it goes ([runtime/docs/calibration.md](runtime/docs/calibration.md)).
- **Where every value comes from** ([runtime/docs/values.md](runtime/docs/values.md)): every configuration value,
  whether it's calibrated, characterized, chosen, or as built, where today's number came from, and
  how to measure it or why it is what it is. A test keeps the register complete: a new value fails
  the build until it has a row. What's measured once per kind of part (a camera's noise and
  latency, a gyro's drift, a tread's grip) has its bench procedure in
  [runtime/docs/characterization.md](runtime/docs/characterization.md).
- **A drive-team dashboard** (`layouts/elastic-layout.json`, for Elastic): a pre-match tab and a
  match tab, checked against the code on every build, so no widget sits blank on the field.
- **AdvantageScope, set up** (`layouts/advantagescope.json`, `ascope_assets/`): open the layout and
  see the robot drawn from its configuration (bumpers, wheels turning as measured, each camera's
  field of view), where it believes it is against where the simulator says it is, each camera's
  accepted and refused tags, and the estimate's ellipse, with nothing to drag in. The build redraws
  the robot whenever `Robots.java` changes
  ([runtime/docs/simulator.md](runtime/docs/simulator.md#see-it-in-advantagescope)).
- **Loop health:** every loop is timed section by section, and alerts say when it overruns (and
  where the time went), when logging falls behind, and when the log drive is missing or filling.
- **Nothing reaches the robot unchecked:** `deploy` runs every check first, and refuses to run
  without a team number.
- **Dependency updates** for build tools and CI actions arrive as weekly pull requests (Dependabot).
  Robot libraries (WPILib, vendordeps) are updated deliberately, per WPILib release.

## Layout

| Path | What |
|---|---|
| `src/main/java/frc/robot/` | Your robot program, the code you write, and nothing else (`frc.robot`): `Main` (hands the runtime which robot this is, this season's field and what the robot does, `RobotProgram.start`), `Robots` (below), `Controls` (your commands, autos and gamepad bindings: the one file you write behavior in, empty until you do, [runtime/docs/mechanisms.md](runtime/docs/mechanisms.md#where-behavior-goes); the guide's robot's buttons come from the runtime's `ExampleControls`), `ToolRobots` (your robots, for the tools the build writes files for), and a team's 2026 robots, `BetaRobot` and `DeltaRobot` with their controls. The build generates `BuildConstants` (the git state), which `Main` passes on. |
| `runtime/` | The runtime, a repository of its own here as a git submodule, built as part of this build (`com.michaelgrundvig.frc.runtime`, `com.michaelgrundvig.frc:frc-robot-runtime`; [runtime/README.md](runtime/README.md)): the robot program every robot shares, which no team writes, and everything it needs to be tested. It reads the libraries below and nothing of `frc.robot`: a team's `Main` hands it what it needs (`RobotProgram`), and `ArchitectureTest` and Gradle keep it so. At its top: `Robot` (logging, Pitwall, and the loop), `RobotContainer` (builds every mechanism on the hardware for the mode, real, simulated, or replayed from a log, and hands them to your `Controls`), `RobotConfig`, `RunMode`, and `ExampleRobot`, the guide's robot, which the runtime is tested on and the template runs until you write your own, with `ExampleControls`, that robot's buttons and autos. Its packages follow. |
| `src/main/java/frc/robot/Robots.java` | Your robots (start from `ExampleRobot`'s blocks): size, mass, drivetrain, cameras, and mechanisms, one typed configuration each, every value set by name and checked when built. Each mechanism is one block: where its joint is, its named positions and outputs, its motors and their CAN IDs, its own sensors, and how this program drives it; the drivetrain's block carries its wiring and gains the same way. The configurations the robot takes and the interlocks between joints name the joints and their places, checked against each other when the robot is built. A practice robot is the competition robot under its own name, with its differences: `COMPETITION.named("practice").with(PIVOT.motors(motors -> motors.on(TALON_FX.can(33))))`. |
| `com.michaelgrundvig.frc.runtime.subsystems` | The mechanisms, in AdvantageKit's IO pattern (the logic, plus one IO class per kind of hardware): `drive`, the swerve drivetrain, simulated and CTRE's (a [stand-in](runtime/docs/stand-ins.md) until AdvantageKit's 2027 swerve template ships); `localization`, where the robot is (it feeds `runtime/lib/localization`'s service, and logs and raises alerts on what it says); `vision`, the cameras; `mechanisms`, every joint the robot's motors drive, each a v3 mechanism built from the robot's definition, with its commands, gates, interlocks and homing ready to use ([runtime/docs/mechanisms.md](runtime/docs/mechanisms.md)). |
| `com.michaelgrundvig.frc.runtime.hardware` | The robot's hardware behind one interface (`Hardware`): the drivetrain's, each joint's, sensor's and camera's IO, and the PDH's, made for the mode the program runs in. On a robot, `RobotHardware`, the devices each block's wiring names; in the simulator, the simulated world's (`com.michaelgrundvig.frc.runtime.sim.SimulatedHardware`); in a replay, nothing, since the log has every reading. |
| `com.michaelgrundvig.frc.runtime.behavior` | What behavior is written with beside Commands v3: gates, named conditions that say why they're false (`Gate`), and the drive team's gamepads, which record each binding with its button (`RecordingGamepad`, `Bindings`). |
| `com.michaelgrundvig.frc.runtime.commands` | Commands that aren't one mechanism's: the calibration procedures. |
| `com.michaelgrundvig.frc.runtime.dashboard` | What the drive team's dashboard shows, and the log stream to it. |
| `com.michaelgrundvig.frc.runtime.sim` | The simulated world's adapters to the robot program: the simulated hardware behind its IO (`SimulatedHardware`: the modules' motor controllers and encoders, the IMUs, the joints, sensors and PDH, the cameras), the faults bound to Pitwall's simulator controls (`Faults`), and the named scenarios (`Scenarios`), over the simulation library (`runtime/lib/simulation`). Only the simulator builds it. |
| `com.michaelgrundvig.frc.runtime.lib` | The robot program's own components, unit-tested and free of robot code (enforced): loop timing (`timing`), Systemcore's CAN ports in WPILib's names (`wiring`), and the Phoenix helper (`phoenix`), which turns a mechanism's definition, wiring and control into its TalonFX and CANcoder settings (ratios, gains, gravity, limits, followers), so nothing is retyped by hand. |
| `runtime/src/test/java/`, `runtime/src/testFixtures/java/` | The runtime's tests, in the same packages as what they test, from single classes to the whole robot in the simulator, on the guide's robot, plus its architecture rules (`ArchitectureTest`). The fixtures are what you test your own robot with: `RobotSim`, `Monkey`, `AutoCheck`, the invariants. See [runtime/docs/testing.md](runtime/docs/testing.md). |
| `src/test/java/` | Your tests: your robots (`robots2026/`), your architecture test (`ArchitectureTest`), and the checks of the dashboard layouts against the code (the guides' and the values register's are the runtime's). |
| `model/` | The robot model, a repository of its own here as a git submodule (`com.michaelgrundvig.frc.world`), read by everything that needs to know the field or the robot: the validation kit every configuration record is built with (`config`), what the field and the robot are both made of, mass, shapes, and materials (`physical`), the field model and its frames, tags, terrain, and zones (`field`), the robot's definition, its body, drivetrain, cameras, IMUs and joints, each with its wiring and control, and what a joint's definition means for a program, as functions of plain values (`robot`), the timed paths a drivetrain follows (`path`), a season's game, what its manual allows a robot to be and its field, written once when the game is revealed (`game`), and the writers that give other tools what they say, such as AdvantageScope's robot assets (`bridges`). It reads only WPILib's math, units, field layouts and JSON library (avaje-jsonb), so Gradle keeps the robot program, the simulator, and Pitwall out of its reach. |
| `localization/` | Localization, a repository of its own here as a git submodule, built as part of this build (`com.michaelgrundvig.frc.localization`, `com.michaelgrundvig.frc:frc-localization`; [localization/README.md](localization/README.md)): the contract a localizer keeps (`Localizer`), the default localizer and WPILib's pose estimator behind it, which a robot chooses between in its configuration ([runtime/docs/localizers.md](runtime/docs/localizers.md)), the service the robot runs each loop (`LocalizationService`: readings and images in, the estimate and a verdict on each image out), the pose estimator and everything that feeds it (its package documentation is a reading map), and the calibration math (`calibration`). It reads the world model and nothing else of the robot's, so its tests run in milliseconds without the simulator. |
| `runtime/lib/simulation/` | The simulation, a project of the runtime's build (`com.michaelgrundvig.frc.simulation`): the physics behind one interface (`Physics`), the IMUs' models, how the simulated robot's sensors err (`SensorErrors`), the catalog of what can fail (`FaultCatalog`), and where each random stream starts (`Seeds`), with the truth suite that checks it against physics ([runtime/docs/simulator.md](runtime/docs/simulator.md#how-far-to-trust-it-the-truth-suite)). It reads the world model and WPILib's math, and nothing of the robot program, localization, or Pitwall; in the robot program only the simulator's adapters (`com.michaelgrundvig.frc.runtime.sim`) and its top level reach it, never the robot's code, which sees its IO alone (the robot program's `ArchitectureTest`). |
| `src/studies/java/` | Your simulation studies that answer design questions, in their own source set so none of their code runs on the robot: where to mount the cameras, and how many ([runtime/docs/camera-placement.md](runtime/docs/camera-placement.md), `./gradlew cameraStudy`); whether the pose estimator's uncertainty tells the truth ([runtime/docs/testing.md](runtime/docs/testing.md#is-the-estimator-honest-the-consistency-study), `./gradlew consistencyStudy`). |
| `pitwall/` | Pitwall, the robot program's own web page and API: status, actions, live tuning, the Robot view's joints and the World view's scenes ([pitwall/README.md](pitwall/README.md)). A repository of its own, here as a git submodule built as part of this build, with its own package (`com.michaelgrundvig.frc.pitwall`), free of robot code. It reads the robot model (`model/`), this build's. The runtime's `Robot` starts it. The model's `docs-processor` copies the configuration records' Javadoc into the program as it compiles, for the page to show. |
| `layouts/` | Dashboard layouts: the drive team's, for Elastic, and AdvantageScope's, for the simulator and its logs. |
| `ascope_assets/` | AdvantageScope's robot models, generated from `Robots.java` by the build: point AdvantageScope's *Use Custom Assets Folder* here. Don't edit them. |
| `runtime/build-logic/` | The runtime's Gradle plugins, one per tool whose files the build generates from the robot's configuration: AdvantageScope's today (`frc.advantagescope-assets`). `settings.gradle` finds them through its `pluginManagement`, and `build.gradle` applies one by its id. |
| `sim/` | WPILib's simulator GUI's default settings, used on the first `./gradlew run -Pgui`. |
| `vendordeps/` | Vendor libraries, as in any WPILib project. |
| `AGENTS.md` | Instructions for coding agents, which `CLAUDE.md` points Claude Code to: what's different from older FRC code, how to check work, and where the guides are. |
| `docs/` | The template's guides: [editors and commands](docs/editors.md), [kickoff](docs/kickoff.md) and [upgrading WPILib](docs/upgrading.md). |
| `runtime/docs/` | The runtime's guides, which document its API: [mechanisms](runtime/docs/mechanisms.md), [driving](runtime/docs/driving.md), [autos](runtime/docs/autos.md), [testing](runtime/docs/testing.md), [the simulator](runtime/docs/simulator.md), [calibration](runtime/docs/calibration.md), [where every value comes from](runtime/docs/values.md), [characterization](runtime/docs/characterization.md), [camera placement](runtime/docs/camera-placement.md), [venues](runtime/docs/venues.md), the [stand-ins](runtime/docs/stand-ins.md), and the [rule requests](runtime/docs/rule-requests.md). |
| `gradle/` | The build's supporting rules: `quality.gradle` (checks), `build-info.gradle` (build metadata), `simulation.gradle` (simulator defaults), `studies.gradle` (the simulation studies), `runtime-versions.gradle` (fails the build when GradleRIO or a vendor library differs from the runtime's), the version catalog, and every downloaded library's checksum (`verification-metadata.xml`, [docs/upgrading.md](docs/upgrading.md#dependency-checksums)). |

## License

[BSD 3-Clause](LICENSE). Parts of `Main.java`, `Robot.java`, `RunMode.java`, and `ModuleIOSim.java` derive from
AdvantageKit's templates (BSD 3-Clause, Littleton Robotics); WPILib is BSD 3-Clause (FIRST and other
WPILib contributors). Their licenses are in [`licenses/`](licenses/).

The libraries the robot program uses come under their own licenses. Only one is in this
repository: uPlot, the plotting library Pitwall's page loads from the robot (no internet at an
event), whose license is in [`licenses/`](licenses/). Gradle downloads the rest, so sharing the
source shares none of them; three.js, which draws Pitwall's World view, is one, and its license is
in [`licenses/`](licenses/) too, since its files go into Pitwall's jar for the page to load. A
built robot program (the jar `deploy` sends) contains them all, so share source, never built jars:

| Library | License |
|---|---|
| WPILib, including Commands v3 | BSD 3-Clause |
| AdvantageKit | BSD 3-Clause |
| PhotonLib | MIT |
| uPlot 1.6.32, in `pitwall/` | MIT |
| three.js 0.181.1, three of its files in Pitwall's jar | MIT |
| PhotonVision's targeting library, which PhotonLib uses | GPL-3.0, whose terms reach a built program you share |
| CTRE Phoenix 6 | CTRE's own license, which doesn't allow redistributing it |
| REVLib | BSD 3-Clause, as the license file in its native libraries says; its Java jar carries no license file |
