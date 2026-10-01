# Testing

Every test runs on every build, locally and in CI, so a test is the cheapest way to keep a bug
fixed. The simulator makes most of the robot testable without a robot: a test can drive it, knock
it, fool its sensors, and check what it believes against where it really is.

## Running tests

| Job | Command |
|---|---|
| Rerun the fast tests on every save while you work | `./gradlew test -Pquick --continuous` |
| One robot test class, whatever its kind | `./gradlew :test --tests frc.robot.RobotContainerSimTest` |
| A package's tests | `./gradlew :test --tests 'frc.robot.lib.field.*'` |
| Pitwall's tests | `./gradlew :pitwall:test` |
| Every test, with the coverage report | `./gradlew test` |
| Every check CI runs | `./gradlew ci` |

The colon in `:test` means the robot project's tests only. Without it, `--tests` also applies to
Pitwall's project, which has no such test and fails the build saying so.

`-Pquick` leaves out the slow tests (below) unless `--tests` names what to run, and skips the static
analysis and coverage report that `ci` runs. The coverage report is at
`build/reports/jacoco/test/html/index.html`.

## Where a test goes

Test classes mirror the package of what they test: the tests for `lib/field/Zone.java` are in
`src/test/java/frc/robot/lib/field/ZoneTest.java`. Pick the lowest level that can show the bug; the
lower the level, the faster the test and the more exactly a failure says what broke.

| Level | What it runs | Example to copy |
|---|---|---|
| A component | Plain JUnit on a class in `lib/`. No simulator, no hardware, no clock: these run in milliseconds, and most tests belong here. | `lib/field/ZoneTest` |
| A mechanism | A subsystem on simulated hardware, stepped loop by loop. The class carries `@ExtendWith(RobotSim.class)`. | `subsystems/drive/DriveBasicsTest`, with `sim/DriveSimHarness` |
| The whole robot | The robot program as `Robot` builds and runs it, Pitwall and all, driven from a simulated Driver Station and gamepad. | `RobotContainerSimTest` |
| The program itself | The robot program started as a program, in its own process: a session recorded, then replayed. | `ReplayEndToEndTest` |
| The structure | The architecture rules (what may depend on what), checked against the compiled code. | `ArchitectureTest` |

Name a test class after what it tests: `ZoneTest`. A test class that runs the simulator for
seconds of robot time is a `...SimTest` and carries `@SimulationTest`; one that starts the program
as a process carries `@EndToEndTest`. Quick mode leaves both out.

## A whole-robot test

`RobotSim` gives each test the same start, whatever ran before it: the clock stopped, the Driver
Station attached and disabled on the blue alliance's station 2, and fresh dashboards. It builds the
robot exactly as `Robot` does, and `loop()` runs exactly the loop `Robot` runs. Afterwards it
closes and resets everything, so nothing leaks into the next test.

```java
@SimulationTest
class DrivingSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void theLeftStickDrivesAwayFromTheDrivers() {
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // pushed forward
    sim.driver().notifyNewData();

    sim.run(1.5); // seconds of robot time, loop by loop

    assertThat(sim.robot().drive().getPose().getX()).isGreaterThan(5.0);
  }
}
```

What it offers:

- **The robot:** `sim.robot()`, built on first use as the simulator's robot with sensors exactly as
  configured. To test another, build it first: `sim.build(SensorErrors.TYPICAL)` for a
  season-worn robot, `sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE)` for odometry alone,
  or `sim.build(Constants.Mode.REAL, robot, SensorErrors.NONE)` for the CTRE code on CTRE's
  simulated devices.
- **Time:** `sim.loop()`, `sim.loops(n)`, and `sim.run(seconds)`. The clock moves only when the test
  steps it, so a test never waits and never depends on how fast the computer is.
- **The Driver Station:** `RobotSim.setEnabled(true)`, `RobotSim.enable(RobotMode.AUTONOMOUS,
  true)`, `RobotSim.station(AllianceStationID.RED_1)`. Change it through these, or call
  `DriverStationSim.notifyNewData()` after changing it yourself, or the change never arrives.
- **The drivers:** `sim.driver()`, the gamepad on port 0.
- **The truth:** `sim.simulation()` knows where the robot really is (`truePose()`), and can move
  it, tilt it, or hit it.
- **Scenarios:** the simulator's named faults and events (`frc.robot.sim.Scenarios`), the ones
  `./gradlew run -Pscenario=<name>` starts. `sim.build(Scenarios.named("camera-low"))` builds the
  robot as `./gradlew run` does, with the scenario's faults on a season-worn robot's sensors
  (`SensorErrors.TYPICAL`, where `sim.robot()` starts from `NONE`), and `sim.start(scenario)` makes
  what happens in it happen (a collision, the gyro unplugged) at the moment the test chooses. `ScenarioSimTest`
  runs every one on the robot. A new scenario goes in `Scenarios.ALL`, with a test in
  `ScenariosTest` that it does to the simulated world what it says.
- **What dashboards see:** `sim.telemetry()`; and the Pitwall page, `sim.pitwall()`: its status,
  its actions, and tuning, as the page uses them.

## Rules that keep tests trustworthy

- **The same run every time.** The clock moves only when stepped, and the simulator's wheel and
  gyro noise comes from a fixed seed, so those runs repeat exactly. PhotonVision's simulated
  cameras add noise of their own, unseeded, so a test with cameras checks against a margin, never
  an exact value. Never `Thread.sleep` to wait for robot time; step the loop.
- **Recorded behavior stays recorded.** `EstimatorGoldenSimTest` compares the pose estimator's
  estimate, loop by loop through a seeded session, with a recording
  (`estimator-golden.csv`). It fails on any change to what the estimator does, which is the point
  while restructuring it. When a change is meant to move the estimate, run the test with the
  environment variable `GOLDEN_UPDATE=1` to record it again, and commit the recording with the
  change, saying why it moved.
- **No test runs past two minutes.** A hung test fails instead of hanging the build. A test that
  really needs longer says so with `@Timeout`.
- **Tests run in `build/test-work`.** What simulated devices write (CTRE's `ctre_sim/`, for one)
  stays out of the project. To read a project file, start from the `frc.projectDir` system
  property, as `DashboardLayoutTest` does.
- **Coverage has a floor.** The build fails below the floors in `coverage-floor.properties`. They
  catch code with no tests at all; they are not a target to chase.

## Configuration is tested too

A bad configuration fails the build, never a robot. Every robot in `Robots.all()` is built and
checked by the tests `deploy` runs first:

- **Its values:** each record checks its own when built, and reports every problem at once
  (`RobotConfigTest` and each record's own test).
- **Its tuning:** every value a mechanism lets the pit tune starts within its limits
  (`RobotTuningTest`).
- **Its rules across mechanisms** (`RobotRules`): the committed configuration keeps every one
  (`RobotRulesTest`).

A new rule gets a test of its own: a change that breaks it is refused, one that keeps it isn't,
and changing both sides together works. Pitwall's `ConfigRulesTest` works one through, two arms
that can collide.
