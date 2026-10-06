# Mechanisms

Describe each mechanism once, as facts about the robot. The template turns every joint your motors
drive into a Commands v3 `Mechanism` with its commands already written: go to a place, run at a
speed, hold, home. You write what the robot *does* with them, in plain Commands v3.

You never write a mechanism class. There's no state machine to learn and no framework between you
and Commands v3. The new words are a joint's **positions** and **outputs**, a **configuration**,
an **interlock**, the **parts** record, and the joint's commands, its **moves** and its **gates**.

The example throughout is an intake: a pivot that swings out, with a roller on its end.

## 1. Describe it

Everything a mechanism *is* goes in its robot's definition, in `Robots.java`: where each joint sits,
its range, what it carries, and the motors that drive it. Two new things go there too: the places
it goes, and the rules between joints.

```java
static final Joint.Revolute PIVOT = Joint.pivot(
    "pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32),   // where its axle is
    Joint.PITCH_DOWN,                                          // it swings down as its angle grows
    Range.degrees(0, 135), Degrees.of(1.5),                    // its hard stops, and where it rests
    Link.box("intake arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.55, 0.36)),
    Position.degrees("in", 1.5), Position.degrees("run", 110), Position.degrees("out", 130));

static final Joint.Revolute ROLLER = Joint.roller(
    "roller", PIVOT, new Translation3d(0, 0, 0.35),            // it rides on the pivot's end
    Link.unseen("roller"),
    Output.rpm("intaking", 2000), Output.volts("outtaking", -4));

static final Drive PIVOT_MOTOR = PIVOT.motors(Motor.KRAKEN_X60, 1, Gearing.of(25));
static final Drive ROLLER_MOTOR = ROLLER.motors(Motor.KRAKEN_X60, 1, Gearing.of(3));
```

- **`Joint.pivot`, `Joint.roller` and `Joint.elevator`** are the common joints in one call each.
  For anything else, such as a pivot with no hard stops, the builders (`Joint.Revolute.builder()`,
  `Joint.Prismatic.builder()`) take every value by name.
- **A link** is what the joint moves. `Link.box` draws it as a box for the extension checks, its
  mass unknown; `Link.weighed` adds a mass from CAD or a scale; `Link.unseen` is neither. A mass
  left out is never guessed: the robot says at startup what it can't work out without it.

- **Positions** are named places a joint goes: "in", "out". They're relative to the joint's
  parent, so an arm's "level" means the same at any elevator height.
- **Outputs** are named ways a joint runs: a speed (`Output.rpm`), a voltage (`Output.volts`) or a
  share of the battery (`Output.dutyCycle`). Every joint also has `"stopped"`.
- **A configuration** is several joints at named places or outputs at once. It names only the
  joints it moves:

  ```java
  static final Configuration FEEDING =
      Configuration.of("feeding", FEEDER.at("feeding"), INDEXER.at("feeding"));
  ```

- **An interlock** is a rule between two joints that must hold whatever command is running:

  ```java
  // The roller runs only once the pivot is past "run".
  static final Interlock ROLLER_ONLY_WHEN_OUT =
      Interlock.that(ROLLER).runsOnlyWhen(PIVOT.past("run"));
  // The arm keeps out of the bumper's band while the elevator is too low to clear it.
  static final Interlock ARM_CLEARS_BUMPER =
      Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.before("clear"));
  ```

  `past("run")` means "run" lies between the joint's reference and where it is now; `before`,
  `between` and `at` complete the set. A sensor's conditions work as well
  (`Interlock.that(ROLLER).runsOnlyWhen(CLAW_BEAM.clear())`), and a lost sensor restricts: a joint
  kept out stays out, one that runs only when it holds is held. Add the configurations and
  interlocks to the robot's definition (`.configurations(...)`, `.interlocks(...)`). The definition
  checks interlocks on joints against each other and the configurations; one on a sensor it can
  only check is on one of the robot's sensors, since what it reads isn't known until the robot runs.

- **Switches and rangefinders**, as the devices they are. A switch is tripped or clear, whatever
  its wiring; its kind says what trips it: a `limit` switch a joint presses at one end of its
  travel, a `beam` break, or a `contact` switch nothing in the model closes (a button, a cage
  sensor). A rangefinder reads a distance along its axis. Each is on a part, the chassis or a
  joint's link (the arm a roller rides on, not the roller, which spins), and where on it is
  optional: a team rarely knows a beam's exact points, and the
  missing-data report says what's a guess.

  ```java
  static final Sensor.Switch PIVOT_DOWN =
      Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, 0);
  static final Sensor.Switch CLAW_BEAM = Sensor.Switch.beam("claw beam", PIVOT);
  ```

  What one means is a condition over it, either way: `CLAW_BEAM.tripped()`, `CLAW_BEAM.clear()`,
  a rangefinder's `nearerThan(Inches.of(6))`. Add them to the definition (`.sensors(...)`). A limit
  switch whose position isn't known can stop its joint but not home it.

**The definition checks itself** when the robot starts, and in the test that builds every robot,
so a mistake fails the build, not a match. It refuses:
- a position outside its joint's range, or a name used twice;
- a speed faster than the joint's motors can turn it, through its gearing (it notes one within 10%
  of it, since the motor can't hold that under load);
- a configuration that breaks an interlock, or two joints whose interlocks wait on each other.

**Name your parts once.** Behavior code shouldn't find joints by their labels: a typo would only
show up on the robot. Instead, give your robot design a small record of its parts, and fill it for
each robot of that design:

```java
@ConfigRecord
record Parts(Joint.Revolute pivot, Joint.Revolute roller, Configuration feeding) {}
```

Your code reads `parts.pivot()`. A part missing or misspelled won't compile. If you build a
practice robot too, fill the same record with its joints, and pick the record the way
`Robots.current()` picks the robot.

## 2. Set it up

How the program drives a joint is its **setup**: one `JointSetup` for each joint your motors drive,
in the robot's `RobotConfig` (`.joints(...)` in `Robots.java`). A setup has three parts:
- **The joint**, from the definition.
- **Its control:** gains, soft limits, motion profile, current limits, neutral mode, bands and
  homing. Every value is in the joint's own units, radians or meters.
- **Its wiring:** the motor controller its motors are on (`TALON_FX`, `SPARK_MAX`,
  `SPARK_FLEX`, or `TALON_FXS`), CAN IDs, motor directions, and the CANcoder if it has one. The
  robot refuses a motor its controller can't drive, saying which can. Two devices of one kind on one
  bus can't share an ID, the drivetrain's included (a TalonFX and a SPARK may): the robot won't
  start, and names both.

```java
static final List<JointSetup> JOINTS = List.of(
    JointSetup.of(PIVOT, MechanismControl.position(30).withProfile(8, 30)),
    JointSetup.of(ROLLER, MechanismControl.velocity(0.02)));
```

`MechanismControl.position(kP)` and `velocity(kP)` start from the defaults (the feedforward from
the definition, 40 A, braking), and each `with...` changes one value; a flywheel adds
`.coasting()`. `JointSetup.of` leaves the joint unwired, which is all the simulator needs; give it
its wiring with `JointSetup.builder()` once the robot exists.

[values.md](values.md) says what each value means and how to find yours. The robot refuses to
start if a driven joint has no setup, or if a named position lies outside its soft limits, where it
would be sent and never get.

- **Gains.** kV, kA and kG start from the definition, worked out from the motors, gearing and mass,
  until you measure them ([characterization](characterization.md)). kP, kI and kD are yours.
- **Profile.** Moves are profiled on the robot: the cruise speed and acceleration of every move. With
  no profile, a joint moves at a quarter of its free speed, safe for a first enable.
- **Bands** (`MechanismControl.Bands`) say how near a target counts as there:
  - `enter` is how close it must come to arrive;
  - `stay` is how far it may then drift and still count.

  The defaults are 2° and 4°, 1 cm and 2 cm, or 3% and 6% of a speed.
- **Live tuning.** Each joint's kP, kD and kS, and its current limits, can be tuned on Pitwall
  while the robot runs (`Joints/<label>/config`); its motor controller takes a change on the next
  loop. Nothing is saved: Pitwall lists what differs from `Robots.java`, with the builder call that
  commits it. Its feedforward, profile and named positions aren't tunable live yet.
- **Homing** is how a joint finds where it is:

  | Recipe | When to use it |
  |---|---|
  | none (empty) | The joint has an absolute encoder in the definition, read through a CANcoder or a SPARK's data port. Nothing to do: it's told where it is at startup. |
  | `Homing.toHardStop(Stop.LOWER)` | No absolute encoder. On the first enable, the joint is pushed gently (2 V) into that hard stop, and its position there becomes the stop's. Moves wait for it; if it finds no stop within 3 s, the joint is faulted and its moves are refused. Check on the robot that 2 V really moves it. |
  | `Homing.toSwitch(Stop.LOWER)` | No absolute encoder, and a limit switch at that end whose position the definition knows. On the first enable, the joint is driven gently (2 V) toward it, and its position as the switch trips becomes the switch's; one starting on it backs off first. It's as close as half the joint's travel in a loop at that speed: slower is closer. Moves wait for it; if the switch doesn't trip within 3 s, the joint is faulted, and says if the switch is lost. |
  | `Homing.assumeAtReference()` | No absolute encoder, and you stow the joint by hand before every power-on. It trusts that. |

  A joint held at positions without an absolute encoder must name one of these: the robot won't
  start until you choose.

A joint with no wiring runs in the simulator. On a robot it stays still, and an alert says so.

**On REV's SPARK MAX and SPARK Flex**, a joint is set up the same way, from the same control: its
SPARK's settings come from the ones a TalonFX would get. A few things differ:
- The SPARK's loop runs kP; the feedforward, gravity and kD are worked out on the robot each loop
  and sent with it as volts.
- A SPARK limits its motor's current (from the control's stator limit), not the battery's.
- An absolute encoder goes on the SPARK's data port (`MechanismWiring.dataPort`), read once at
  startup; a SPARK doesn't read a CANcoder. kI isn't carried to a SPARK yet. The robot says so if a
  setup has either.
- Its settings are saved on the SPARK at startup, so a brownout doesn't lose them.

**A sensor's wiring** is in the robot's `RobotConfig` too (`.sensors(...)`), one for each switch
and rangefinder the robot has, by its label: a Systemcore port, a CANdi's input, a limit input on
a joint's motor controller (a SPARK's, a TalonFXS's or a Falcon's; a Kraken has none), or a
CANrange; and for a switch, its polarity, normally open or normally closed, which
nothing above the wiring sees. Wire a limit switch normally closed: a cut wire then reads tripped,
and its joint stops. The robot refuses a wiring for a sensor it doesn't have, two sensors on one
input, a controller's limit input where there's none, and a joint's limit switch on another
joint's controller, which would stop the wrong motor.

**The controller's own stop.** A joint's limit switch on its controller's limit input stops the
motor in the controller itself, with no code in the loop; so does one on a CANdi on a TalonFX's
bus, which the TalonFX reads over CAN as its limit (a Kraken has no limit inputs of its own). The
controller stops on nothing else: a beam on a SPARK's input is read, never stopped on, and an input
with nothing wired stops nothing. Neither resets the joint's position as it trips: homing is the
robot's. And beneath every command and interlock, the robot's code stops a joint at any of its
tripped limit switches, wherever it's wired: a move past one holds where it tripped
(`HeldBy`: "pivot down tripped"), an output toward it goes neutral, and leaving it is free.

A limit switch is checked against where its homed joint is. Tripped nowhere near where it trips,
or clear well past it (a tenth of the joint's range either way, for a quarter second), it's
doubted, in the log (`Joints/<label>/SwitchDoubts`) and an alert, saying what to check: "pivot
down reads tripped with pivot at 130.0°, nowhere near where it trips, 30.0°: a cut wire, if it's
normally closed, or a switch out of place". A doubted switch still stops its joint. In the
simulator, `world.sensors().cut(switch, polarity)` cuts its wire.

```java
SensorWiring.OfSwitch.builder()
    .sensor("claw beam")
    .input(new SensorWiring.Candi(CAN, 30, 1))
    .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
    .build()
```

## 3. Use it

Look a joint up by its part, and use its commands. Every one is ready to bind or await:

```java
JointMechanism pivot = joints.of(parts.pivot());
JointMechanism roller = joints.of(parts.roller());
```

| Command | What it does | Ends |
|---|---|---|
| `pivot.goTo("out")` | Moves to a named position, profiled from where it is | When it's there (inside its band for 40 ms), or at its deadline |
| `pivot.goTo("out").at(0.5)` | The same at half its cruise speed | As above, with a longer deadline |
| `pivot.goTo(Degrees.of(118))` | Moves to a computed position; refused outside its limits | As above |
| `pivot.goTo("out").within(Seconds.of(1))` | The same with a tighter deadline | As above |
| `hood.follow(() -> Degrees.of(angleFor(distance())))` | Chases a target that changes every loop, a position or a speed | Never: cancel it |
| `pivot.hold()` | Owns the joint and holds its goal | Never |
| `roller.run("intaking")` | Runs at a named output | Never: for "while held" |
| `roller.runUntil("intaking", hasPiece)` | Runs until a condition holds | When it holds, or after 3 s |
| `pivot.home()` | Homes it again by its recipe | When it's homed |
| `joints.goTo(parts.stowed())` | Moves a configuration's joints together; requires exactly those joints | When every one is there |
| `joints.run(parts.feeding())` | Holds a configuration, outputs included | Never |

A move's **default deadline** comes from the model: twice its profile's time from where it starts,
plus half a second. A move that can't get there doesn't hang your auto: it times out, and says
where it was.

**Every move ends with an outcome**, and a reason that's already logged:

| Outcome | Means |
|---|---|
| `ARRIVED` | It got there, or its condition held |
| `TIMED_OUT` | Its deadline passed. The reason says where the joint was, and what interlock held it |
| `CANCELED` | Another command took the joint, its button was let go, or its `onlyWhile` gate fell |
| `REFUSED` | It wouldn't start: a target outside the limits, a joint that isn't answering or failed to home, or its `onlyIf` gate false |

A configuration's move also says when each joint got there, so the slow one is plain:
`"→ intaking: ARRIVED in 0.62 s (pivot 0.58 s, roller 0.10 s, elevator 0.62 s)"`, and
`move.arrivals()` gives the same as numbers.

Recovery is an `if`:

```java
Command.requiring(pivot).executing(co -> {
  var deploy = pivot.goTo("out");
  co.await(deploy);
  if (!deploy.arrived()) {
    co.await(pivot.goTo("in"));    // back off
  }
}).named("Deploy or back off");
```

**When nothing drives it, a joint idles:**
- **A joint held at positions holds its goal:** where its last move arrived, or, if that move was
  canceled or timed out, where it came to a stop. It never sags, and never snaps back to an old
  target.
- **A roller stops.**
- **A default command replaces the idle**, as in any v3 code:
  `hood.setDefaultCommand(hood.goTo("down"))`.

**Interlocks hold beneath every command.** No command, button or auto can skip one. While
`runsOnlyWhen` doesn't hold, the joint stays where it is, or a roller stops. While `keepsOut` is
closed, the joint's target stops at the region's edge on the side it's on. The command keeps
running and carries on once the interlock lets go, so moving joints together is safe without
ordering them yourself. If the joint an interlock reads hasn't homed, or isn't answering, the
interlock assumes the worst and holds.

**A law of your own**, such as a push held to a current, is a command that sets the joint's
voltage every loop. The joint goes back to its idle the loop you stop:

```java
roller.run(co -> { while (true) { roller.setVoltage(pushVolts()); co.yield(); } }).named("Push");
```

### Where behavior goes

Write what the robot does in one class: its intake, its shot, its autos. Build it in
`RobotContainer`, after the joints, and bind it there:

```java
final class Actions {
  private final JointMechanism pivot, roller;

  Actions(Joints joints, Parts parts) {
    pivot = joints.of(parts.pivot());
    roller = joints.of(parts.roller());
  }

  /** Deploy and roll. The interlock holds the roller until the pivot passes "run". */
  Command intake() {
    return Command.requiring(pivot, roller).executing(co -> {
      co.fork(pivot.goTo("out"));
      co.await(roller.run("intaking"));
    }).named("Intake");
  }
}

// In RobotContainer:
operator.rightTrigger().whileTrue(actions.intake());
operator.faceRight().onTrue(pivot.goTo("in"));
```

Each driven joint is its own v3 `Mechanism`, so a command requires exactly the joints it moves.
Deploying while the shooter spins is fine, and one button can't cancel another's unrelated work.

## 4. Gates: wait for things, and know why not

A `Gate` is a v3 trigger with a name, and it logs why it's false. Bind it, await it, or use it to
guard a move. Every joint has gates already:

| Gate | True when |
|---|---|
| `pivot.isAt("out")` | It's within its band of a named position, or runs at a named output |
| `pivot.isPast("run")`, `pivot.isBetween("run", "out")` | Where it is, measured |
| `shooter.isAtGoal()` | It's where it's asked to be: at its position, or within its band of its speed |
| `pivot.isReady()` | It knows where it is, and its motor controllers answer |
| `indexer.isStalled()` | It's run at an output but has moved slower than a tenth of it for a quarter second |
| `joints.isAt(parts.stowed())` | Every joint of a configuration is at its place |

Make your own from anything:

```java
Gate aimed = Gate.within(scheduler, "aimed", () -> headingErrorDegrees(), 2.0, 4.0);  // in at 2°, out past 4°
Gate readyToShoot = Gate.all("ready to shoot", shooter.isAtGoal(), aimed)
    .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING);     // rides out a flywheel's dip
```

Make gates where you build your commands, not inside one that runs over and over.

**A sensor's conditions are gates too.** The robot reads its switches and rangefinders each loop,
with the joints (`joints.sensors()`, logged under `Sensors/<label>`), and any condition on one
becomes a gate, named for what it means:

```java
Gate coralIn = sensors.gate("coral in claw", parts.clawBeam().tripped());
Gate hopperFull = sensors.gate("hopper full", parts.hopperRange().nearerThan(Inches.of(6)));

roller.runUntil("intaking", coralIn)    // ends ARRIVED the loop it trips; TIMED_OUT after 3 s
co.waitUntil(coralIn);                  // holds a step
coralIn.onTrue(countPiece());           // once each time it trips
coralIn.debounce(Milliseconds.of(40), Debouncer.DebounceType.RISING)  // a beam that flickers
```

A sensor that's lost (its CAN device silent, or no wiring on a robot) holds no condition either
way: neither tripped nor clear. A gate on it never opens and says so ("claw beam lost"), the alert
names it, and a move waiting on it times out saying why: "roller: intaking until coral in claw:
TIMED_OUT after 3.00 s: coral in claw is false: claw beam lost". In the simulator a limit switch
trips where its joint really is; a test says what a beam or a rangefinder sees
(`world.sensors().trip(...)`, `see(...)`, and `silence(...)` for a lost one).

Guard a move with one:

```java
pivot.goTo("out").onlyIf(intakeClear)             // checked once: REFUSED if false
joints.run(parts.feeding()).onlyWhile(readyToShoot)  // checked every loop: CANCELED when it falls
```

"Feed whenever ready, pause when not" is a loop:

```java
Command shoot() {
  return Command.requiring(shooter, feeder, indexer).executing(co -> {
    co.fork(shooter.follow(() -> RPM.of(speedFor(distance()))));
    while (true) {
      co.waitUntil(readyToShoot);
      co.await(joints.run(parts.feeding()).onlyWhile(readyToShoot));
    }
  }).named("Shoot");
}
```

One v3 trap: a binding made *inside* a command with `whileTrue` on a gate that's already true never
starts, because `whileTrue` only starts on a change. Use the loop above, or `retryWhileTrue`.

## 5. Why won't it move?

Every joint logs under `Joints/<label>/`, every sensor under `Sensors/<label>`, and every gate
under `Gates/<name>`. Open them in
AdvantageScope or Pitwall, and read them in this order:

| Key | Tells you |
|---|---|
| `State` | `READY`, `UNHOMED`, `HOMING`, or `FAULTED` and why: a homing that found no stop, or a motor controller that didn't take its settings at startup, which nothing then drives |
| `Command` | The command driving it this loop, or `idle` |
| `HeldBy` | The interlock holding it, if one is |
| `Goal`, `GoalName` and `At` | Where it's headed, and where it is in its names: "between run and out" |
| `Outcome` | How its last move ended: "pivot → out: TIMED_OUT after 1.60 s, pivot at 40.0°, held: …" |

"Why isn't it shooting?" is `Gates/ready to shoot/Blocking`: the terms that are false.

## 6. Test it in the simulator

Every driven joint is simulated from its definition: the motors through the gearing, what the
joint carries, gravity on it, and its hard stops. Behavior can be tested without a robot.

- **One mechanism alone:** `JointsSimHarness` runs the joints, the scheduler and nothing else:

  ```java
  JointsSimHarness sim = new JointsSimHarness(definition, setups);
  Move out = sim.joints.of(PIVOT).goTo("out");
  sim.run(out, 1.5);
  assertThat(out.arrived()).isTrue();
  assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
  ```

- **The whole robot:** `RobotSim` runs it, drivetrain and gamepads included ([testing](testing.md)).

`src/test/java/frc/robot/robots2026/` has six real 2026 robots described this way. One of them runs
end to end in the simulator, gamepads, interlock, gates and auto included (`Team2611Actions` and
`Team2611RobotSimTest`).

## Not yet

- **Joints the template doesn't drive yet:** a differential's pair, and servos. The robot says
  which, and why. An interlock on one is refused, since nothing could enforce it. A
  configuration naming one is refused whenever it would start, saying why, and the startup's
  doubts list it, so the rest of the robot still runs.
- **A bench bypass for interlocks.** It waits for a pit mode.
- **Moving between configurations by a route,** such as first raising the arm so the elevator can
  drop. Until then, write the two steps.
- **Aiming the drivetrain, and following paths,** belong to the drive. The path tool is still to be
  chosen.
