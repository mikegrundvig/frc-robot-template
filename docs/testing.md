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
| The tests of Pitwall's documentation processor | `./gradlew :pitwall:docs-processor:test` |
| Every test, with the coverage report | `./gradlew test` |
| Every check CI runs | `./gradlew ci` |

The colon in `:test` means the robot project's tests only. Without it, `--tests` also applies to
Pitwall's projects, which have no such test and fail the build saying so.

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
- **Faults and scenarios:** what can fail in the simulated world while it runs
  (`frc.robot.sim.Faults`), applied as the Pitwall page applies them: `sim.components().apply(
  Setting.of(Faults.GYRO, Faults.UNPLUGGED))`, and `undo` for one that lasts. A scenario is a named
  set of them (`frc.robot.sim.Scenarios`), the ones `./gradlew run -Pscenario=<name>` starts.
  `sim.build(Scenarios.named("camera-low"))` builds the robot as `./gradlew run` does, with the
  scenario's sensor faults on a season-worn robot's sensors (`SensorErrors.TYPICAL`, where
  `sim.robot()` starts from `NONE`), and `sim.start(scenario)` makes what happens in it happen (a
  collision, the gyro unplugged) at the moment the test chooses. `FaultsTest` and
  `CameraFaultsSimTest` show each fault changes what the robot's code reads, and undoing it puts
  that back; `ScenarioSimTest` runs every scenario on the robot. A new fault goes in `Faults` with
  such a test; a new scenario goes in `Scenarios.ALL`, with a test in `ScenariosTest` that it does
  to the simulated world what it says.
- **What dashboards see:** `sim.telemetry()`; and the Pitwall page, `sim.pitwall()`: its status,
  actions, tuning, and experiments, as the page uses them. The page's own requests go over HTTP to
  `sim.pitwall().port()` (use `127.0.0.1`); `SteeringExperimentSimTest` drives an experiment the
  way the page does, its event stream and all. Status items are read only when a page watches or
  asks, so ask first (`sim.pitwall().status().requestRead()`). The experiments' deadman runs on
  the robot's clock, so it steps with the loops: send the page's heartbeats every so many loops,
  not on a timer.

## Rules that keep tests trustworthy

- **The same run every time.** The clock moves only when stepped, and the simulator's wheel and
  gyro noise and its cameras' noise (one seed per camera, in `VisionSim`) come from fixed seeds,
  so a run repeats exactly. A test with cameras still checks against a margin, never an exact
  value: a change anywhere else moves the noise it draws. Never `Thread.sleep` to wait for robot
  time; step the loop. To wait for the network (an answer, an event on a stream), wait on it with
  a timeout, a loop at a time; to show nothing else arrived, send a change and check it's the next
  thing to.
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
  property, as `DashboardLayoutTest` does, and declare the file an input of the test task in
  `build.gradle`, so Gradle knows the tests depend on it. Today the tests run on every build
  anyway (the build's time is compiled in), but if that changes, Gradle would call them up to
  date after a change to an undeclared file, and skip them.
- **Coverage has a floor.** The build fails below the floors in `coverage-floor.properties`. They
  catch code with no tests at all; they are not a target to chase.

## Is the estimator honest? The consistency study

The pose estimator reports how uncertain it is, and the χ² gate, the scoring gate's uncertainty
limit, and any later bound on its error all trust that number. A claim of 3 mm while the robot is
10 cm off is worse than no claim at all. The simulator knows where the robot really is, so it can
check: `./gradlew consistencyStudy` drives the whole simulated robot through seven runs, twelve
seeds of each, and compares every loop's estimate with the truth (`frc.robot.studies`:
`ConsistencyStudy`, `ConsistencyRuns`, `Consistency`). It takes about five minutes, one robot at a
time (WPILib's simulation is one per program). The report is in `build/studies/consistency/`.

The runs, on the simulator's season-worn robot (`SensorErrors.TYPICAL`) with four cameras:

| Run | What happens |
|---|---|
| `driving` | The route in view of the tags, there and back, then 3 s standing |
| `standing` | The route once, then 20 s standing still |
| `turning` | 5 s standing, two turns in place, 5 s standing |
| `gyro-scale` | The gyro-scale procedure (five turns), then standing: 40 s |
| `collision` | The `collision` scenario partway through the route |
| `camera-low` | The `camera-low` scenario, driving the route |
| `camera-turned` | The `camera-turned` scenario, driving the route |

A scenario's sensor faults are applied through the simulator's controls before the robot's first
loop, as `./gradlew run -Pscenario=<name>` applies them, and its events when the run's script
says; a knocked camera is knocked from where the seed mounted it.

Each seed draws its own: noise in every camera and IMU, an error for each camera's mount (along
and about each axis), one for each tag (along each axis: the cameras see a tag where it is, the
robot believes the layout), and the gyro's and the backup IMU's scales. Each kind of error, and
each camera and tag, draws from its own stream of the seed, so no two seeds share a draw. They're
drawn from Gaussians at the spread the estimator's configuration claims (`EstimatorConfig`'s
`cameraTurnStdDevDegrees`, `cameraShiftStdDevMeters`, `headingStdDevDegreesPerTurn`), with the
vision noise model halved, which is how noisy the simulated cameras are: an estimator honest about
its own claims reads a NEES near its number of dimensions. Wider or narrower errors than it claims
show it over- or under-confident, which is what the options are for.

| Option | Default | What |
|---|---|---|
| `--runs=A,B` | all | Which runs |
| `--seeds=N` | 12 | Runs of each, each with its own draws |
| `--seed=N` | 1 | The first seed |
| `--mount-error-mm=M`, `--mount-error-degrees=D` | 3, `cameraTurnStdDevDegrees` | Each camera's mount off along and about each axis (standard deviations) |
| `--tag-error-mm=M` | `cameraShiftStdDevMeters` | Each tag off the layout along each axis; 0 for an exact field |
| `--gyro-scale-error=F`, `--backup-scale-error=F` | `headingStdDevDegreesPerTurn` / 360, 0.005 | Each IMU's scale off by this share |
| `--noise-scale=F` | 0.5 | The vision noise model scaled; 1 is the model as shipped, twice the simulated cameras' noise |
| `--out=DIR` | `build/studies/consistency` | Where the report goes |

The console prints each seed's draws and numbers, so a run's result can be traced to the seeds
behind it.

### Reading the report

`summary.md` has a row a run. Each column answers a question:

- **NEES** (normalized estimation error squared): the error weighted by the inverse of the
  covariance the estimate claimed. An honest estimator averages its number of dimensions: 2 for
  position, 1 for heading. Far above, it's overconfident: sure of an estimate that's off. Below,
  it's timid: it trusts its measurements less than it could, which costs accuracy but misleads
  nobody.
- **Above, below:** at each loop, the NEES averaged over the seeds falls inside a 95% band (1.03
  to 3.28 for position with 12 seeds) if honest, so about 2.5% of loops land above it and 2.5%
  below. Many above is overconfidence at those moments, even when the average looks fine.
- **Outside the ellipse:** how often the position error falls outside the claimed covariance's
  99.9% ellipse (its NEES past 13.8, χ² for two dimensions). Honest is a thousandth of loops.
- **Past 3.72 σ:** how often the error goes past 3.72 times the claimed spread along its long
  axis: the circle a bound on the error, such as a scoring gate's, would use. It holds the ellipse,
  so it's past less often; the ellipse also catches an estimate too sure along its short axis.
- **Allowed:** the most of either an honest estimator shows, 999 times in 1000. A seed's draws
  last its whole run, so a seed that drew a gyro 2.4 σ off can spend a quarter of its loops outside
  the ellipse: the seeds, not the loops, are the sample, and twelve allow 2.9%.
- **Worst:** the most long-axis spreads the error ever was.
- **Mean error, worst error, heading error, claimed σ:** accuracy beside honesty. An estimator
  that got honest by getting worse is no improvement.

`loops.csv` has each run's NEES, error, and claimed spread loop by loop, averaged over the seeds,
for a spreadsheet: it shows when a run goes wrong (during the turns, or standing after them).

What to expect, today (12 matched seeds):

| Run | NEES (2) | Heading NEES (1) | Outside the ellipse | Past 3.72 σ | Mean error | Heading error |
|---|---|---|---|---|---|---|
| driving | 1.54 | 0.76 | 0 | 0 | 1.6 cm | 0.22° |
| standing | 1.85 | 0.43 | 0 | 0 | 1.3 cm | 0.17° |
| turning | 1.46 | 0.70 | 0 | 0 | 1.6 cm | 0.69° |
| gyro-scale | 2.07 | 0.95 | 0.10% | 0.01% | 1.8 cm | 1.35° |
| collision | 2.76 | 0.85 | 0.57% | 0.57% | 1.8 cm | 0.21° |
| camera-low | 1.60 | 0.83 | 0 | 0 | 1.7 cm | 0.24° |
| camera-turned | 3.40 | 14.17 | 2.45% | 2.20% | 2.5 cm | 0.98° |

Every calibrated run is near its NEES, a little below: the floor for what a camera repeats is
sized for the worst of what it allows. The collision's exceedances are the loops of the hit,
before the jolt is read. The `camera-turned` run's heading NEES is far above: a camera turned 2°
is eight times what the configuration allows, and the estimator can't tell it from a heading
error. Catching that is the mount check's job, and comparing cameras with each other would do it
better.

### What it proves, and what it doesn't

It proves the estimator's arithmetic keeps its claims honest against the errors the simulator
makes: wheel wear, a gyro's scale and drift, the cameras' noise, mounts and tags within their
tolerances, a collision. It can't prove the real robot's numbers: that the noise model fits the
real cameras, that the gyro's scale is as configured, that the field's tags are within half an
inch. Those are measured ([characterization](characterization.md)), and on a robot the check is
the same idea without the truth: a camera's normalized innovations (NIS), and the error at
surveyed spots. A configured repeated error smaller than the real one shows here too: with
`cameraShiftStdDevDegrees` at 1°, half what tags 1 cm off put in a heading, the gyro-scale run read
a heading NEES of 2.05 and had 4.9% of its loops outside the ellipse.

### Keeping it honest

`EstimatorConsistencySimTest` runs four of the runs (driving, standing, turning, the gyro-scale
procedure), six matched seeds each, on every build, about 75 s. Each must land where six honest
seeds do 999 times in 1000: its mean position NEES between 0.30 and 5.8 and its heading NEES
between 0.04 and 4.1, its share of loops outside the ellipse and past 3.72 σ within what six seeds
allow (4%), and its mean error under 2.5 to 3.5 cm. Six seeds can't pin a NEES down closely, so the
test catches an estimator that's badly over- or under-confident, as the one before this study was
(a NEES of 17 to 476 on these draws); the full study is the closer look. A change to the
estimator, its configuration, or the vision pipeline that breaks one has made the estimate claim
more or less than it knows, or got there by being worse. Run the whole study before and after such
a change, and put both tables in the commit.

## Configuration is tested too

A bad configuration fails the build, never a robot. Every robot in `Robots.all()` is built and
checked by the tests `deploy` runs first:

- **Its values:** each record checks its own when built, and reports every problem at once
  (`RobotConfigTest` and each record's own test).
- **Its tuning:** every value a mechanism lets the pit tune starts within its limits
  (`RobotTuningTest`).
- **Its rules across mechanisms** (`RobotRules`): the committed configuration keeps every one
  (`RobotRulesTest`).
- **Its register:** every configuration value, and every value a mechanism tunes, has a row in
  [docs/values.md](values.md) saying what kind it is and where its number came from, and no row
  names one that's gone (`ValuesRegisterTest`).

A new rule gets a test of its own: a change that breaks it is refused, one that keeps it isn't,
and changing both sides together works. Pitwall's `ConfigRulesTest` works one through, two arms
that can collide.
