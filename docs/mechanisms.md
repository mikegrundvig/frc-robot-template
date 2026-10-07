# Mechanisms

Describe each mechanism once, in one place. The template turns every joint your motors drive into
a Commands v3 `Mechanism` with its commands already written: go to a place, run at a speed, hold,
home. You write what the robot *does* with them, in plain Commands v3.

You never write a mechanism class. There's no state machine to learn and no framework between you
and Commands v3. The new words are a joint's **block**, its **positions** and **outputs**, a
**configuration**, an **interlock**, and the joint's commands, its **moves** and its **gates**.

The example throughout is an intake: a pivot that swings out, with a roller on its end.

## 1. Describe it, in one block

Everything about a mechanism goes in one block in `Robots.java`: where its joint sits, its range,
what it carries, the places it goes, its motors and their wiring, its own sensors, its interlocks,
and how the program drives it. The places come first, as constants, so your code names them, and a
misspelled one won't compile.

```java
static final Position PIVOT_IN = Position.degrees("in", 1.5);
static final Position PIVOT_OUT = Position.degrees("out", 130);

static final Joint.Revolute PIVOT =
    Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
        .range(Range.degrees(0, 135), PIVOT_IN)          // its hard stops, and where it rests
        .carries(Link.box("intake arm", new Translation3d(0, 0, 0.2),
            new Translation3d(0.05, 0.55, 0.36)))
        .positions(PIVOT_IN, PIVOT_OUT)
        .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32).pdh(4, Amps.of(40)))
        .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))
        .control(MechanismControl.position(30)
            .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));

static final Output INTAKING = Output.rpm("intaking", 2000);
static final Output OUTTAKING = Output.volts("outtaking", -4);

static final Joint.Revolute ROLLER =
    Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))   // it rides on the pivot's end
        .outputs(INTAKING, OUTTAKING)
        .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31).pdh(5, Amps.of(40)))
        .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))   // only once the pivot is out of the frame
        .control(MechanismControl.velocity(0.02));
```

Then list the joints in the robot's definition: `.joints(List.of(PIVOT, ROLLER))`.

- **`Joint.pivot`, `Joint.roller` and `Joint.elevator`** start the common joints; each call after
  says one thing about it. For anything else, the builders (`Joint.Revolute.builder()`,
  `Joint.Prismatic.builder()`) take every value by name: first what every joint has, then what it
  may not (its range, where it rests, its places, what drives it), each left out if it hasn't.
- **A link** is what the joint moves. `Link.box` draws it as a box for the extension checks, its
  mass unknown; `Link.weighed` adds a mass from CAD or a scale; `Link.unseen` is neither. A mass
  is optional, and a rough one does: the simulator reads it, and beyond that it only seeds a
  joint's kG and kA until you tune them ([Gravity](#gravity)). One left out is never guessed: the
  simulator says what it does without it.
- **Positions** are named places a joint goes: "in", "out". They're relative to the joint's
  parent, so an arm's "level" means the same at any elevator height. Code names each by its
  constant, and goes where the robot running it puts it, so a practice robot's own "out" is where
  its pivot goes (below).
- **Outputs** are named ways a joint runs: a speed (`Output.rpm`), a voltage (`Output.volts`) or a
  share of the battery (`Output.dutyCycle`). Every joint also has `Output.STOPPED`. Two joints
  that both have a "feeding" each get their own constant: `FEEDER_FEEDING`, `INDEXER_FEEDING`. A
  constant is its joint's at the joint's own value, so the indexer's, at another speed, is refused
  on the feeder; two at one speed may share one.
- **Motors** turn it through its gearing (`.motor(...)`, or `.motors(...)` for several together,
  the leader first), and onto a drum for one that slides (`.motors(Motor.KRAKEN_X60, gearing,
  drumRadius, ...)`). With no controllers named, it runs in the simulator alone. A drive that
  turns two joints at once, a differential's, is the robot's own (`.drives(...)` in the
  definition).
- **Its own sensors**: an absolute encoder turning with it (`.absoluteEncoder(...)`, see
  [Absolute encoders](#absolute-encoders)), and the limit switches it presses at the ends of its
  travel (`.limitSwitch(End.LOWER, Degrees.of(0), ...)`). [Sensors](#sensors) has the rest.
- **Its interlocks** hold it by where another joint is, beneath every command: the roller runs only
  with the pivot out (`.runsOnlyWhen(...)`, below).

**A block says each thing once.** What a joint has one of (its range, where it rests, its link,
its positions, its outputs, its motors, its absolute encoder, a switch at each end, its control) is
refused said twice, saying how to change it: a second `.absoluteEncoder(...)` meant as a pair is
"pivot already has an absolute encoder: to re-wire it, encoderOn(...); a decoded pair isn't
supported yet". What it has several of, servos and interlocks, adds up.

**How it's driven isn't part of what the joint is, nor where its places are.** A joint re-tuned
or re-wired is the same joint, and so is one whose places are placed otherwise: a position or an
output is a name on its joint, and where it is, or how fast, the robot's. Everything that names
the joint or its places still finds them, the robot's own. So a practice robot of the same design
is the competition robot, `COMPETITION` in `Robots.java`, under its own name, with its
differences, and the same code runs on either, `pivot.goTo(PIVOT_OUT)` to each robot's own out:

```java
static final RobotConfig PRACTICE =
    COMPETITION.named("practice")
        .with(
            PIVOT.motorsOn(TALON_FX.can(33).pdh(4, Amps.of(40))) // another CAN ID
                .replaced(PIVOT_OUT, Degrees.of(128)), // it swings out to 128°
            ARM.encoderOffset(Rotations.of(0.47)) // its magnet's own offset
                .retuned(control -> control.withKg(0.3)) // a lighter arm
                .rest(Degrees.of(-30))) // it stows at -30°
        .with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen())) // a beam on another CANdi
        .with(CameraConfigBuilder.builder(BACK_LEFT).up(Inches.of(20.5)).build()); // lower
```

Each change keeps the rest of what it changes, so a CAD revision that regears the competition
robot's pivot regears the practice robot's too:
- `motorsOn(...)` puts a joint's motors on other controllers, the leader first, keeping the motor,
  how many and their gearing; `gearing(...)` gears them otherwise, keeping their controllers.
- `encoderOffset(...)` gives its absolute encoder another offset, its magnet's own, keeping its
  CANcoder, data port or Systemcore port and the rest of their wiring, so a CANcoder moved to
  another ID on the competition robot moves on the practice robot too; `encoderOn(...)` reads it
  on another device, keeping its ratio and counts.
- `limitSwitchOn(End.LOWER, ...)` puts its switch at that end on another input, keeping where it
  trips.
- `retuned(control -> ...)` changes its control, a wither at a time.
- `replaced(PIVOT_OUT, Degrees.of(128))` puts one of its named positions elsewhere, its name kept,
  so a move, a configuration or an interlock that names it finds it there: a practice elevator
  that scores at its own heights. `replaced(UNJAMMING, Volts.of(-1))` runs one of its outputs
  otherwise, at a speed, a voltage, or a share of the battery, `Percent.of(60)`. A name the joint
  doesn't have is refused: its places' names are what its code says, the same on every robot.
  The joint keeps where each was, so the competition robot's constant is still its own; one of the
  name at a value it has never had is another joint's or another robot's, and refused (below).
- `range(Range.degrees(-355, 5))` moves its hard stops, keeping where it rests, which must be
  inside them: a practice turret that turns less far than the competition robot's.
- `rest(Degrees.of(-30))` moves where it rests as the robot starts, keeping its stops, which it
  must be inside: a practice arm that stows elsewhere. Given one of its positions,
  `rest(ARM_STOWED)`, it rests at the joint's own, so after `replaced(ARM_STOWED, ...)`, at the
  practice robot's. A rest given in its block as a position stays where it was when that position
  is re-placed: move it too. Every check runs on it: the interlocks as the robot starts there, and
  `past` and `before`, measured from where it rests, move with it (`atLeast` and `atMost` don't).
- A sensor of the robot's own is re-wired by `.on(...)`. The drivetrain's modules are offset by
  its `encoderOffsets(...)`, front left, front right, back left, back right, as Tuner X gives them,
  keeping the rest of its wiring, `.with(DRIVETRAIN.encoderOffsets(Rotations.of(0.12), ...))`, and
  re-wired whole by its `wired(...)`. Its modules may differ in what only the simulator reads,
  Tuner X's inertias, and still be its own.
- A camera mounted otherwise is given whole, through its builder, and replaces the robot's camera
  of its name, as the back-left one above, half an inch lower.

Some differences have no change yet: a drivetrain re-geared or its modules placed otherwise, a
joint on another count of motors, and a joint only one of the robots has. Write those through the
builders for now, `RobotDefinitionBuilder.builder(COMPETITION.definition()).joints(...)`, knowing a
later change to that part of the competition robot won't reach the practice robot.

Every check of the definition runs on the practice robot's values: a place outside its range, an
interlock's edge or a soft limit past its stops, or a place where a configuration breaks an
interlock, is refused. They're checked together, on the finished joint, so its changes go in any
order: a practice wrist whose stops and places both move, each past the other's, is
`WRIST.range(...).replaced(...)` either way round. A joint given twice is refused, since the
second would drop the first's change: chain them, as the pivot's and the arm's are.
`PIVOT.motor(...)` in a practice robot, which would restate its gearing and leave it behind when
the competition robot's changes, is refused as a second say of its motors: "pivot already has its
motors: to re-wire them, motorsOn(...); to re-gear them, gearing(...)". List the practice robot in
`Robots.all()` beside the competition robot, `List.of(COMPETITION, PRACTICE)`, and the test that
builds every robot checks it.

**Configurations** name joints and their places, so they come after the blocks; an **interlock** is
usually written in the block of the joint it restricts:

- **A configuration** is several joints at their places or outputs at once. It names only the
  joints it moves:

  ```java
  static final Configuration FEEDING =
      Configuration.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING));
  ```

- **An interlock** is a rule between two joints that must hold whatever command is running: one
  per physical clash, written in the block of the joint it restricts:

  ```java fragment
  static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))
          ...
          // The roller runs only once the pivot is out of the frame.
          .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
          .control(MechanismControl.velocity(0.02));

  static final Joint.Revolute ARM =
      Joint.pivot("arm", ELEVATOR, new Translation3d(0.1, 0, 0.3), Joint.PITCH_UP)
          ...
          // The arm keeps out of the bumper's band while the elevator is too low to clear it.
          .keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Inches.of(32)))
          .control(...);
  ```

  An edge is usually a clearance, a number from CAD or measured where the parts just clear, not a
  place the joint goes, so it's written as one: `PIVOT.atLeast(Degrees.of(110))` holds with the
  pivot at 110° or beyond, `ELEVATOR.atMost(Inches.of(32))` with the elevator at 32 in or below,
  and `between` inside a band. They mean the same wherever the joint rests, so moving its rest in a
  CAD revision leaves them alone. `past` and `before` are measured from where the joint rests
  (`PIVOT.past(PIVOT_RUN)` holds from "run" on, away from its rest), for an edge that really is a
  named position: `ELEVATOR.before(ELEVATOR_SCORE)`. `SHOOTER.atLeast(RPM.of(3000))` holds a joint
  until another is up to speed. A sensor's conditions work as well
  (`.runsOnlyWhen(CLAW_BEAM.clear())`), and a lost sensor restricts: a joint kept out stays out,
  one that runs only when it holds is held.

  A block can only name joints written above it, so a joint restricted by one that rides on it (an
  elevator held up while its own arm is down) has its rule in the robot's definition instead:
  `.interlocks(List.of(Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(ARM.atMost(Degrees.of(-40)))))`,
  written the same way. Either place is fine; both are enforced and checked together, and a rule
  written in both is refused. The definition checks interlocks on joints against each other and
  the configurations; one on a sensor, or on a speed, it can only check is the robot's, since
  neither is known until the robot runs.

  Two joints may restrict each other, as the arm and the elevator above do, so long as one can
  always move to free the other: kept below the bumper's band, the arm can still rise to its edge
  at -10°, above -40°, and let the elevator down. Rules that hold each joint where only the other
  could free it are a deadlock, and the definition refuses them where the robot starts or a
  configuration puts it, saying where each is held. The elevator kept at its bottom while the arm
  is at most 0°, with the arm resting below the band, is one: "as the robot starts, it leaves
  elevator and arm holding each other by their interlocks, so neither can free the other: a
  deadlock (elevator keeps out of 0.1 m to 0.6 m while arm at most 0°, so elevator stays between
  0 m and 0.1 m; arm keeps out of -10° to 30° while elevator at most 0.8128 m, so arm stays between
  -30° and -10°)".

  A joint may rest inside a region it's kept out of, as an elevator at its bottom stop kept out of
  its lowest stretch while the intake is in does. It's a doubt said at startup, not a refusal:
  until the region opens, the joint may only leave it, by the edge with room past it (the nearer,
  if both have), and the definition's deadlock check counts that way out. A configuration may not
  put a joint there.

**The definition checks itself** when the robot starts, and in the test that builds every robot,
so a mistake fails the build, not a match. It refuses:
- a position, or where its joint rests, outside the joint's range; a name used twice;
- a position, an output, or a configuration's or a condition's place that isn't that joint's own:
  a name it hasn't, or one of its names at a value it has never had;
- a speed faster than the joint's motors can turn it, through its gearing, whether one of its
  outputs or its profile's cruise (it notes an output within 10% of it, since the motor can't hold
  that under load);
- a value of the other kind: a range, soft limits, a profile, bands or gains given in angles for a
  joint that slides, or in lengths for one that turns;
- a control that doesn't fit its joint: soft limits past its hard stops, or a named position
  outside them; a homing recipe it can't carry out (a roller's, a hard stop on one that turns
  freely, a switch it hasn't, or one whose position isn't known); a joint held at positions with
  neither an absolute encoder nor a recipe; a loop closed on an absolute encoder where it can't be
  ([Absolute encoders](#absolute-encoders));
- wiring that can't work: an encoder on a TalonFX's data port, which it hasn't; two sensors on one
  input; a switch on a motor controller's limit input that has none, or another joint's;
- a configuration that breaks an interlock, joints whose interlocks hold each other for good where
  the robot starts or a configuration puts them, or an interlock written twice. A joint resting
  inside a region it's kept out of is a doubt, not a refusal.

## 2. How it's driven and wired

### Control

`.control(...)` says how the program drives the joint: gains, soft limits, motion profile, current
limits, neutral mode, bands and homing. `MechanismControl.position(kP)` and `velocity(kP)` start
from the defaults (the feedforward from the definition, 40 A, braking), and each `with...` changes
one value; a flywheel adds `.coasting()`. A joint with motors needs one: the robot won't start
without it, and says what to give.

Each value is given with its unit, or as a plain number in the joint's own units, radians or
meters: `withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700))` or
`withProfile(7.85, 29.7)`, `withSoftLimits(Range.degrees(5, 130))`, `withBands(Degrees.of(1),
Degrees.of(2))`. A kP of 30 is 30 V per radian of error on a pivot, and per meter on an elevator;
one per turn, as Phoenix's slot holds it, is written with its unit,
`position(Volts.per(Rotation).of(188))`, as is SysId's kV per turn per second,
`withKv(Volts.per(RotationsPerSecond).of(0.12))`. A value with its unit knows whether it's an angle
or a length, so an elevator's soft limits in degrees are refused, saying how to write them; plain
soft limits refused as too large say when they look like degrees, and a plain profile refused as
too fast says it's in radians per second, and how to give it with its unit.

[values.md](values.md) says what each value means and how to find yours. The definition refuses
a named position outside its soft limits, where it would be sent and never get.

How the program drives the joint is worked out once from its block: its gains, tuned or seeded,
its gravity, its limits, and the one feedforward law (kS, kV, kA and kG) that every loop driving
it runs, its motor controller's, the robot's own, and the simulator's. Its motor controller's
settings are that, in the controller's own terms, worked out as the robot's configuration is
checked: when the robot starts, in the simulator, and in the test that builds every robot. What the
controller can't take is refused there, in the words the robot would use: a gravity angle past
Phoenix's quarter turn, a ratio or a gain past what Phoenix holds, a kI on a SPARK. They're worked
out for the controller the joint is wired to, and for a joint with no wiring, for a TalonFX, which
is what the simulator runs it as. What's doubtful, though the controller takes it (a tuned kG far
from what the joint's weight needs, a gravity offset at Phoenix's limit), is said as the robot
starts, with the definition's other doubts: in the log (`Robot/Doubts`), in the
simulator as on the robot, and on the robot's Driver Station console.

- **Gains.** kV, kA and kG start from the definition, worked out from the motors, gearing and mass,
  until you measure them ([characterization](characterization.md)): `withKs`, `withKv`, `withKa`
  and `withKg` take what you measure. kP, kI and kD are yours. kG is the one that matters most for
  a joint held against gravity: [Gravity](#gravity).
- **Profile.** Moves are profiled on the robot, by WPILib's `TrapezoidProfile`: the cruise speed and
  acceleration of every move. With no profile, a joint moves at a quarter of its free speed, safe
  for a first enable.
- **Bands** (`MechanismControl.Bands`) say how near a target counts as there:
  - `enter` is how close it must come to arrive;
  - `stay` is how far it may then drift and still count.

  The defaults are 2° and 4°, 1 cm and 2 cm, or 3% and 6% of a speed:
  `withBands(Degrees.of(1), Degrees.of(2))` changes them, and a roller's, which are shares of the
  speed it's asked for, `withBands(0.03, 0.06)`.
- **Live tuning.** Each joint's kP, kD and kS, and its current limits, can be tuned on Pitwall
  while the robot runs (`Joints/<label>/config`); its motor controller takes a change on the next
  loop. Nothing is saved: Pitwall lists what differs from `Robots.java`, each with the call that
  commits it in the joint's block: `.withKs(0.25)`, `.withSupplyLimit(Amps.of(50.0))`, and a kP as
  its factory, `MechanismControl.position(31.5)`. Its feedforward, profile and named positions
  aren't tunable live yet.
- **Homing** is how a joint finds where it is:

  | Recipe | When to use it |
  |---|---|
  | none (empty) | The joint has an absolute encoder in its block. Nothing to do: it's told where it is at startup, from a CANcoder or a SPARK's data port by its motor controller, or from a Systemcore port by the robot (below). |
  | `Homing.toHardStop(Stop.LOWER)` | No absolute encoder. On the first enable, the joint is pushed gently (2 V) into that hard stop, and its position there becomes the stop's. Moves wait for it; if it finds no stop in time (below), the joint is faulted and its moves are refused. Check on the robot that 2 V really moves it. |
  | `Homing.toSwitch(Stop.LOWER)` | No absolute encoder, and a limit switch at that end whose position its block knows. On the first enable, the joint is driven gently (2 V) toward it, and its position as the switch trips becomes the switch's; one starting on it backs off first. It's as close as half the joint's travel in a loop at that speed: slower is closer. Moves wait for it; if the switch doesn't trip in time (below), the joint is faulted, and says if the switch is lost. |
  | `Homing.assumeAtReference()` | No absolute encoder, and you stow the joint by hand before every power-on. It trusts that. |

  A joint held at positions without an absolute encoder must name one of these: the definition
  refuses it until you choose.

  **How long homing may take** comes from the joint: twice the time its motors, through its
  gearing, take to cross its whole range at 2 V, and half a second more, as a move's deadline is.
  A tall elevator geared slow gets the time it needs, and a quick pivot isn't left pushing for long
  on a stop it never finds. A joint that turns freely, homed to its switch, gets a turn's worth.
  While it homes, its `State` says the time ("HOMING: toward its lower stop, for at most 9.5 s"),
  and a fault says it and where it came from: "elevator's homing found no stop in 9.5 s (its
  travel takes 4.5 s at 2.0 V), pushing toward its lower one". A joint slower than its model, one
  pushed up against its weight, say, takes a time of its own: `Homing.toHardStop(Stop.UPPER,
  Seconds.of(4))`.

### Gravity

An arm or an elevator is held up by its **kG**: the volts its motors push with to hold it still,
which you tune on the robot and write in its control. It needs no mass:

```java fragment
.control(MechanismControl.position(30).withKg(0.35))                   // held with 0.35 V
.control(MechanismControl.position(30).withKg(0.35, Degrees.of(-90)))  // pulling hardest at -90°
```

- **An elevator's kG** holds it the same all along its travel.
- **An arm's kG** holds it where its weight pulls hardest, and less as it turns, along one cosine
  (Phoenix's `Arm_Cosine`, WPILib's `ArmFeedforward`). Where that is: the angle you give with its
  kG; else where its link's mass puts it, if it's weighed; else its zero, which WPILib takes as
  level. An arm zeroed level, raised as its position grows, needs no angle. One zeroed straight up
  and tipping forward is `Degrees.of(-90)`: it pulls hardest a quarter turn back. To find it, let
  the arm hang still, its motors coasting, and read its angle: add 90° (hanging at -90°, it's 0°).
  Phoenix takes an angle within a quarter turn of the joint's zero; past that, the robot's
  configuration is refused, in the simulator too, saying how to turn the joint's axis around.
- **A counterbalanced joint**, a gas spring or surgical tubing holding it up, writes `withKg(0)`.
- **Without a kG written**, a joint's kG is seeded from its links' mass: none without one. A joint
  held at positions that gravity acts on (an arm whose axle isn't vertical, an elevator; not a
  roller or a turret) with no kG written is a doubt the robot says at startup, on the robot and
  in the simulator: "pivot has no kG: it holds against gravity by kP alone, so it sits below its
  targets until its kG is tuned and written". A kG seeded from a mass is said too, since only a
  tuned one is known to hold it.
- **In the simulator**, a joint's gravity is what its kG holds, through its motors, so a joint
  with a tuned kG holds there as it does on the robot, mass or none; without one, its links'
  weight. Its inertia comes from its links' mass, and from its motors' rotors alone without one.
  What the simulator does without a mass it says when it starts (`Robot/SimulatorNotes`), and
  never on the robot.

### Wiring

Each motor's controller is named where the motor is: its kind (`TALON_FX`, `SPARK_MAX`,
`SPARK_FLEX`, or `TALON_FXS`), its CAN ID, and, on another bus, the bus
(`TALON_FX.can(CANIVORE, 23)`). Add `.clockwise()` for a motor that turns clockwise, seen facing
its shaft, when the joint moves forward; a follower turned the other way from its leader is one of
these. The robot refuses a motor its controller can't drive, saying which can, and a joint's motors
on different kinds of controller or different buses. Two devices of one kind on one bus can't
share an ID, the drivetrain's included (a TalonFX and a SPARK may): the robot won't start, and
names both.

**Power.** Each motor controller says which of the PDH's channels feeds it, and the breaker in that
channel, where it's wired: `TALON_FX.can(32).clockwise().pdh(4, Amps.of(40))`, channel 4 as labeled
on the PDH, through a 40 A breaker. The PDH itself is the robot's, declared once in its definition,
`.pdh(Pdh.can(1))`: REV's Power Distribution Hub on Systemcore's CAN, at its ID, 1
unless it's been changed. The drivetrain's motor controllers say theirs in its wiring, each module's
`driveFeed` and `steerFeed`. The robot refuses a channel that feeds two devices, one the PDH hasn't
(it has 0 to 23), a breaker over its channel's rating (40 A on 0 to 19, 15 A on 20 to 23), and a
feed on a robot with no PDH. With a PDH declared, a wired motor controller that doesn't say its
channel is a doubt said at startup. Both are optional: a robot that declares no PDH says nothing of
feeds. CTRE's Power Distribution Panel isn't supported yet.

The robot reads the PDH each loop, with the joints: each channel's current, their total and the
battery's voltage (`PDH/ChannelCurrentAmps`, `PDH/TotalCurrentAmps`, `PDH/Volts`); each device it
feeds by name (`PDH/Devices/pivot motor 32`, `PDH/Devices/front-left drive`); and each joint's
motors together (`Joints/<label>/PdhCurrentAmps`), beside its leading controller's own reading
(`Joints/<label>/SupplyCurrentAmps`), which is its motor's alone. A PDH that doesn't answer is an
alert. In the simulator, each channel reads what the simulated motor it feeds draws, and a test can
zero one (`world.pdh().zero(4, true)`), as a tripped breaker leaves it.

**A motor drawing nothing** is flagged: a follower whose breaker tripped, so its leader drives the
joint alone and nothing else says so, or a controller its block puts on the wrong channel. While a
joint works, its leading controller drawing more than 2 A from the battery by its own reading, each
of its motors draws about as much on its own channel; one whose channel reads under 0.5 A for half
a second is flagged, in `Joints/<label>/DeadChannels` and an alert, saying what to check: "arm motor
12 draws nothing on channel 5 while arm's motors work: its breaker tripped, its power is unplugged,
or its block names the wrong channel". The joint's control says otherwise,
`.withDeadChannel(Amps.of(3), Amps.of(0.5), Seconds.of(1))`. What it's asked doesn't count: a roller
at a steady speed with no load draws next to nothing, alive or not. A leader that loses power stops
answering on CAN, and its joint's moves are refused saying so; a lone motor unplugged from a SPARK,
which then draws nothing whatever it's asked, isn't flagged here.

A joint with no wiring runs in the simulator on the controller its motor runs on, its control
checked as that controller would take it: a NEO's a SPARK MAX, a NEO Vortex's a SPARK Flex, a
Minion's a TalonFXS, any other's a TalonFX. On a robot it stays still, and an alert says so.

**On REV's SPARK MAX and SPARK Flex**, a joint is driven the same way, from the same control: its
SPARK's settings are what a TalonFX's are, in a SPARK's terms, and only what a SPARK can't take is
refused. A few things differ:
- The SPARK's loop runs kP; the feedforward, gravity and kD are worked out on the robot each loop
  and sent with it as volts.
- A SPARK limits its motor's current (from the control's stator limit), not the battery's.
- An absolute encoder goes on the SPARK's data port
  (`.absoluteEncoder(DATA_PORT.offset(Rotations.of(0.82)))`), read once at startup, or the SPARK's
  loop closed on it ([Absolute encoders](#absolute-encoders)); a
  SPARK doesn't read a CANcoder. kI isn't carried to a SPARK yet. The robot's configuration refuses
  a joint on a SPARK with either, in the simulator too.
- Its settings are saved on the SPARK at startup, so a brownout doesn't lose them.

### Absolute encoders

An absolute encoder knows where the joint is the moment the robot starts, so a joint with one needs
no homing. It goes in the joint's block, read one of three ways, and the joint uses it one of two
ways:

- **Seeded once**, as each is unless its wiring says otherwise: the encoder tells the motor's own
  encoder where the joint is at startup, and the motor's encoder runs the joint's loop from then
  on. Backlash, or a belt that slips, between the motor and the joint then moves where the joint
  goes.
- **Closed on it**: the loop runs on the encoder itself, so the joint goes where the encoder says,
  whatever the motor's count does.

| Where it's plugged in | In the block | Seeded once | Closed on it |
|---|---|---|---|
| A CANcoder | `.absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))`, on its motors' bus | The robot reads it at startup and tells the joint's TalonFX | `.remote()`, `.fused()` or `.sync()` after its offset: the TalonFX's own loop, at 1 kHz |
| A SPARK's data port, such as REV's Through Bore | `.absoluteEncoder(DATA_PORT.offset(Rotations.of(0.82)))` | The SPARK, read at startup | `DATA_PORT.offset(...).closesLoop()`: the SPARK's own loop, at the controller's rate |
| A Systemcore port: a duty-cycle encoder, or an analog encoder or potentiometer | `.absoluteEncoder(PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)))`, or `PortEncoder.analog(2, 10)` for a ten-turn pot | The robot, from its first good reading | `PortEncoder.dutyCycle(1).offset(...).closesLoop()`: the robot's own loop, each loop, at 50 Hz |

With no wiring (`.absoluteEncoder()`), only the simulator reads it.

**What closing on it costs.** A motor controller's loop runs at 1 kHz; the robot's runs once a
loop, at 50 Hz, twenty times coarser: a kP a motor controller holds steady may make it buzz, so tune
it again, lower if it does. The motor's own encoder still gives the joint's speed, which is
smoother than an absolute encoder's. Only a joint held at positions, without a homing recipe and
with a range, closes its loop on its encoder: the robot refuses one that turns freely, since the
encoder's reading wraps within its travel.

**Closed on a SPARK's data-port encoder**, the SPARK runs its loop on the encoder (its feedback
sensor), in the encoder's turns: REVLib 2027 has no conversion factors. The robot works out the
rest, as for any joint on a SPARK: a position goes to the SPARK as the encoder's turns, the kP as a
share of 12 V per turn of the encoder rather than of the rotor (so the same control's kP, on an
encoder on the axle of a pivot geared 25:1, is 25 times what it is per rotor turn), the soft limits
in the encoder's turns, and the feedforward in volts, with the joint's speed from the motor's own
encoder. The SPARK is told the encoder's direction and offset, and reads zero at the low edge of
the window one turn of the encoder tells apart, so its reading never wraps inside the joint's
range, and REV's position wrapping stays off: on, it would take the short way round, through a hard
stop. Where the joint is, logged and for every move, is the encoder's word. None of this is
confirmed on a SPARK yet: that it applies its offset and direction, and its soft limits, to the
sensor its loop runs on, as REV documents, and what it does with an encoder that's unplugged. REV's
simulation runs its loop on the motor's own encoder whatever it's told, and in the template's
simulator a joint on a SPARK runs as a SPARK, its kP on the device and its feedforward the robot's,
but its loop on its motor's count, so a slip the encoder would correct isn't simulated.

**Closed on a Systemcore port encoder**, the robot works out the motor's voltage each loop from
where the encoder says the joint is: the control's kP and kD, with the feedforward a SPARK gets
(kS, kV, kA and kG), sent as a voltage. The profile, the interlocks, the holds and the outcomes are
a joint's as always, and where the joint is, for every one of them, is the encoder's word. Its
motor's count is still told where the joint is at startup and by `mechanism.home()`, and its motor
controller's soft limits stay on that count. A duty-cycle encoder that's lost faults the joint, as
a motor controller gone silent does: its moves are refused, those running end saying why ("pivot's
absolute encoder is lost, on port 1: ..."), and the interlocks that read it hold. It doesn't go
limp: it holds where it comes to a stop, by its motor's count, on its motor controller's own loop,
until the encoder's first reading makes it ready again. An analog encoder can't say it's lost: a
loose wire reads as somewhere, and a loop closed on it drives the joint there.

**Geared**: an encoder that isn't on the joint's axle is written in full, with its turns for each
of the joint's: `.absoluteEncoder(AbsoluteEncoder.of().ratio(0.536).on(CANCODER.can(34).offset(...)))`. Where the joint
is, is known within one of the encoder's turns; the robot says if that window doesn't cover the
joint's travel.

**Counts per turn** are optional. Only a pair of encoders on one joint geared nearly alike reads
them, a turret's on 17 and 18 teeth decoded by their difference, and the robot refuses that pair
without each one's, from its datasheet: `.absoluteEncoder(4096, CANCODER.can(34).offset(...))`,
or `AbsoluteEncoder.of(4096)`.

**Offsets.** Each encoder's offset makes it read zero with the joint at zero, the pose its origin
gives in CAD. A team's code often holds one already: a CANcoder's `MagnetOffset` is its offset as
it is, and a `SensorDirection` of `Clockwise_Positive` is `CANCODER.can(...).clockwise()`. A SPARK's
`zeroOffset` is the raw reading at zero, before inverting, so it's the negative here,
`DATA_PORT.offset(-zeroOffset)`, or `DATA_PORT.offset(+zeroOffset).inverted()`; from it, the template tells the SPARK
a `zeroOffset` of its own, at the edge of the encoder's window. The same goes for WPILib's
`DutyCycleEncoder` expected zero.

**A Systemcore port encoder, on the robot.** Until its first reading, the joint waits, unhomed, and
moves on it wait. With none in 2 s it's faulted and says so; it's still seeded when a reading
comes. `mechanism.home()` seeds it again from the encoder, if the motor's count slipped. Its
reading is logged raw (`Joints/<joint>/Absolute`). A duty-cycle encoder says when it's unplugged;
an analog one can't.

**Backlash and slip show in the log.** Where an encoder on a Systemcore port or a SPARK's data port
says the joint is, is logged each loop (`Joints/<joint>/AbsoluteAt`), beside where the motor
controller says (`Joints/<joint>`'s position). A gap between them that follows the joint's
direction of travel is backlash, and one that grows or jumps is slip: either is a reason to close
the loop on the encoder. Closed on a SPARK's encoder, the joint's position is the encoder's.

### Sensors

**A joint's limit switches** are in its block: `.limitSwitch(End.LOWER, Degrees.of(0),
Direction.REVERSE.normallyClosed())` on its own controller's limit pin, or with a Systemcore port
(`SensorWiring.port(1).normallyClosed()`) or a CANdi's input (`CANDI.can(30).s1().normallyClosed()`)
in place of the pin. Each is named for its joint and end,
"pivot lower limit", and `PIVOT.switchAt(End.LOWER)` is the switch, for a condition. One whose
position isn't known (`.limitSwitch(End.LOWER)` without an angle) can stop its joint but not home
it.

**Every other switch and rangefinder** is a constant of its own, on its part, the chassis or a
joint's link (the arm a roller rides on, not the roller, which spins), carrying its wiring:

```java
static final Sensor.Switch CLAW_BEAM =
    Sensor.Switch.beam("claw beam", PIVOT).on(CANDI.can(30).s1().normallyOpen());
static final Sensor.Rangefinder HOPPER =
    Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
        .on(CANRANGE.can(31));
```

A device on a CANivore names its bus first, as a motor's controller does:
`CANDI.can(CANIVORE, 30).s1()`.

Add them to the definition (`.sensors(...)`). A switch is tripped or clear, whatever its wiring;
its kind says what trips it: a `beam` break, or a `contact` switch nothing in the model closes (a
button, a cage sensor). A rangefinder reads a distance along its axis. Where on its part it sits is
optional: a team rarely knows a beam's exact points, and the missing-data report says what's a
guess. What one means is a condition over it, either way: `CLAW_BEAM.tripped()`,
`CLAW_BEAM.clear()`, a rangefinder's `nearerThan(Inches.of(6))`.

A switch's wiring is where it's connected (a Systemcore port, a CANdi's input, or a limit input on
a joint's motor controller: a SPARK's, a TalonFXS's or a Falcon's; a Kraken has none) and its
polarity, normally open or normally closed (`.normallyOpen()` or `.normallyClosed()` after the
input), which nothing above the wiring sees. Wire a limit
switch normally closed: a cut wire then reads tripped, and its joint stops. A sensor with no wiring
runs in the simulator alone. The definition refuses two sensors on one input, a controller's limit input
where there's none, and a joint's limit switch on another joint's controller, which would stop the
wrong motor.

**The controller's own stop.** A joint's limit switch on its controller's limit input stops the
motor in the controller itself, with no code in the loop; so does one on a CANdi on a TalonFX's
bus, which the TalonFX reads over CAN as its limit (a Kraken has no limit inputs of its own). The
controller stops on nothing else: a beam on a SPARK's input is read, never stopped on, and an input
with nothing wired stops nothing. Neither resets the joint's position as it trips: homing is the
robot's. And beneath every command and interlock, the robot's code stops a joint at any of its
tripped limit switches, wherever it's wired: a move past one holds where it tripped
(`HeldBy`: "pivot lower limit tripped"), an output toward it goes neutral, and leaving it is free.

A limit switch is checked against where its homed joint is. Tripped nowhere near where it trips,
or clear well past it (a tenth of the joint's range either way, for a quarter second), it's
doubted, in the log (`Joints/<label>/SwitchDoubts`) and an alert, saying what to check: "pivot
lower limit reads tripped with pivot at 130°, nowhere near where it trips, 30°: a cut wire, if
it's normally closed, or a switch out of place". A doubted switch still stops its joint. In the
simulator, `world.sensors().cut(switch, polarity)` cuts its wire.

## 3. Use it

Look a joint up by its constant, and use its commands. Every one is ready to bind or await:

```java
JointMechanism pivot = joints.of(PIVOT);
JointMechanism roller = joints.of(ROLLER);
```

| Command | What it does | Ends |
|---|---|---|
| `pivot.goTo(PIVOT_OUT)` | Moves to one of its positions, profiled from where it is | When it's there (inside its band for 40 ms), or at its deadline |
| `pivot.goTo(PIVOT_OUT).at(0.5)` | The same at half its cruise speed | As above, with a longer deadline |
| `pivot.goTo(Degrees.of(118))` | Moves to a computed position; refused outside its limits. A turret takes the nearest turn of it inside them (below) | As above |
| `pivot.goTo(PIVOT_OUT).within(Seconds.of(1))` | The same with a tighter deadline | As above |
| `hood.follow(() -> Degrees.of(angleFor(distance())))` | Chases a target that changes every loop, a position or a speed | Never: cancel it |
| `pivot.hold()` | Owns the joint and holds its goal | Never |
| `roller.run(INTAKING)` | Runs at one of its outputs | Never: for "while held" |
| `roller.runUntil(INTAKING, hasPiece)` | Runs until a condition holds | When it holds, or after 3 s |
| `pivot.home()` | Homes it again by its recipe | When it's homed |
| `joints.goTo(STOWED)` | Moves a configuration's joints together; requires exactly those joints | When every one is there |
| `joints.run(FEEDING)` | Holds a configuration, outputs included | Never |

A move's **default deadline** comes from the model: twice its profile's time from where it starts,
plus half a second. A move that can't get there doesn't hang your auto: it times out, and says
where it was.

**A position or an output is a name on its joint**, and a move goes where the robot running it
puts it: on a practice robot, `pivot.goTo(PIVOT_OUT)` goes to its own out. **A name the joint
doesn't have**, another joint's or one left out of its `.positions(...)`, fails where it's built as
the robot starts: `pivot.goTo(ELEVATOR_CLEAR)` in a binding, or in an auto built with them, stops
the robot starting and says why, so the test that builds every robot fails too. So does one of
its names at a value the joint has never had: with a feeder "feeding" at 1000 rpm and an indexer at
1500, `feeder.run(INDEXER_FEEDING)` is refused, "feeder's feeding runs at 1000 rpm, and this
feeding, at 1500 rpm, isn't one it has had: another joint's or another robot's; give feeder's
own". The competition robot's constant is its practice robot's own wherever that robot re-placed
it, and two joints' places of one name and value, an elevator's and a hopper's "down" at 0 m, are
each one's, whichever constant names them. A configuration or a condition naming another joint's
place is refused the same way, where it's written. Built once the robot runs, in a command or
anywhere else, a move is `REFUSED` as it starts, with the same reason: a robot on the field is
never stopped by one.

**On a turret**, a joint whose range spans a whole turn or more (-200° to 200°, say), an angle
your code works out is a heading: the turret can face it at more than one position. `goTo` and
`follow` take whichever turn of it, the angle plus or minus whole turns, lies inside its limits
(its soft limits, else its range) nearest where the turret is now. At 170°, asked for -170°, it
goes on to 190°, not back round; asked for a heading it can reach only the long way, it goes the
long way. So `turret.follow(() -> bearing.minus(heading))` needs no wrapping of your own. A named
position goes exactly where it's named: a team that named -180° meant -180°. Soft limits narrower
than a turn leave some headings with no turn inside: a move there is refused, and a chase holds at
the limit nearer the heading. A joint narrower than a turn, an arm or a hood, takes an angle as it
is: one off by whole turns is a slip (degrees given as radians), and is refused. A joint with no
range at all turns freely and goes the short way round.

**Every move ends with an outcome**, and a reason that's already logged:

| Outcome | Means |
|---|---|
| `ARRIVED` | It got there, or its condition held |
| `TIMED_OUT` | Its deadline passed. The reason says where the joint was, and what interlock held it |
| `CANCELED` | Another command took the joint, its button was let go, or its `onlyWhile` gate fell |
| `REFUSED` | It wouldn't start: a target outside the limits, a place not the joint's own, a joint that isn't answering or failed to home, or its `onlyIf` gate false |

A configuration's move also says when each joint got there, so the slow one is plain:
`"→ intaking: ARRIVED in 0.62 s (pivot 0.58 s, roller 0.10 s, elevator 0.62 s)"`, and
`move.arrivals()` gives the same as numbers.

Recovery is an `if`:

```java
Command.requiring(pivot).executing(co -> {
  var deploy = pivot.goTo(PIVOT_OUT);
  co.await(deploy);
  if (!deploy.arrived()) {
    co.await(pivot.goTo(PIVOT_IN));    // back off
  }
}).named("Deploy or back off");
```

**When nothing drives it, a joint idles:**
- **A joint held at positions holds its goal:** where its last move arrived, or, if that move was
  canceled or timed out, where it came to a stop. It never sags, and never snaps back to an old
  target.
- **A roller stops.**
- **A default command replaces the idle**, as in any v3 code:
  `hood.setDefaultCommand(hood.goTo(HOOD_DOWN))`.

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

Write what the robot does in one class, `Controls` (`src/main/java/frc/robot/Controls.java`): its
intake, its shot, its autos, and the drive team's buttons for them. It's yours: the template never
changes it, so the template's updates never collide with your code. The robot builds it once,
after its mechanisms, and hands it everything behavior reaches: the joints, the drivetrain, the
sensors, and two gamepads, the driver's on port 0 and the operator's on port 1. Write each command
as a method, and bind it in the constructor:

```java
public final class Controls {
  private final JointMechanism pivot, roller;

  Controls(Scheduler scheduler, Joints joints, Drive drive, Sensors sensors,
      RecordingGamepad driver, RecordingGamepad operator) {
    pivot = joints.of(PIVOT);
    roller = joints.of(ROLLER);

    operator.rightTrigger().whileTrue(intake());
    operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));
  }

  /** Deploy and roll. The interlock holds the roller until the pivot passes 110°. */
  Command intake() {
    return Command.requiring(pivot, roller).executing(co -> {
      co.fork(pivot.goTo(PIVOT_OUT));
      co.await(roller.run(INTAKING));
    }).named("Intake");
  }
}
```

Each driven joint is its own v3 `Mechanism`, so a command requires exactly the joints it moves.
Deploying while the shooter spins is fine, and one button can't cancel another's unrelated work.

The driver's sticks drive the robot, field-relative from the driver's end of the field, and the
driver's back button, with the robot facing straight downfield, resets its heading: the template
binds those itself, in `RobotContainer`, so they're there whatever `Controls` does.

**The robot knows its bindings.** The gamepads are WPILib's `CommandGamepad`, and bind as it does;
each binding is recorded as it's made, with its button and its command's name: "operator right
trigger: while held: Intake", and one made with a gate, "operator right bumper and ready to shoot:
while held: Shoot" (any other condition is "an unnamed condition": make it a gate to name it). The
robot logs the list (`Controls/Bindings`), and each loop the bindings a press or a release started
(`Controls/Fired`). Two commands bound to one button the same way start together, which is
sometimes meant: the robot says so as it starts, with its other doubts (`Robot/Doubts`). A binding
made inside a running command lives only while that command runs, and isn't in the list.

## 4. Gates: wait for things, and know why not

A `Gate` is a v3 trigger with a name, and it logs why it's false. Bind it, await it, or use it to
guard a move. Every joint has gates already:

| Gate | True when |
|---|---|
| `pivot.isAt(PIVOT_OUT)` | It's within its band of one of its positions, or runs at one of its outputs |
| `pivot.isPast(PIVOT_OUT)`, `pivot.isBetween(PIVOT_IN, PIVOT_OUT)` | Where it is, measured |
| `shooter.isAtGoal()` | It's where it's asked to be: at its position, or within its band of its speed |
| `pivot.isReady()` | It knows where it is, and its motor controllers answer |
| `indexer.isStalled()` | It's run at an output but has moved slower than a tenth of it for a quarter second |
| `joints.isAt(STOWED)` | Every joint of a configuration is at its place |

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

```java fragment
Gate coralIn = sensors.gate("coral in claw", CLAW_BEAM.tripped());
Gate hopperFull = sensors.gate("hopper full", HOPPER.nearerThan(Inches.of(6)));

roller.runUntil(INTAKING, coralIn)      // ends ARRIVED the loop it trips; TIMED_OUT after 3 s
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

```java fragment
pivot.goTo(PIVOT_OUT).onlyIf(intakeClear)          // checked once: REFUSED if false
joints.run(FEEDING).onlyWhile(readyToShoot)       // checked every loop: CANCELED when it falls
```

"Feed whenever ready, pause when not" is a loop:

```java
Command shoot() {
  return Command.requiring(shooter, feeder, indexer).executing(co -> {
    co.fork(shooter.follow(() -> RPM.of(speedFor(distance()))));
    while (true) {
      co.waitUntil(readyToShoot);
      co.await(joints.run(FEEDING).onlyWhile(readyToShoot));
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
| `State` | `READY`, `UNHOMED`, `HOMING` toward what and for how long at most, or `FAULTED` and why: a homing that found no stop, or a motor controller that didn't take its settings at startup, which nothing then drives |
| `Command` | The command driving it this loop, or `idle` |
| `HeldBy` | The interlock holding it, if one is |
| `Goal`, `GoalName` and `At` | Where it's headed, and where it is in its names: "between run and out" |
| `Outcome` | How its last move ended: "pivot → out: TIMED_OUT after 1.60 s, pivot at 40.3°, held: …" |

"Why isn't it shooting?" is `Gates/ready to shoot/Blocking`: the terms that are false.

## 6. Test it in the simulator

Every driven joint is simulated from its definition: the motors through the gearing, what the
joint carries, gravity on it (what its kG holds, [Gravity](#gravity)), and its hard stops.
Behavior can be tested without a robot.

- **One mechanism alone:** `JointsSimHarness` runs the joints, the scheduler and nothing else:

  ```java
  JointsSimHarness sim = new JointsSimHarness(definition);
  Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);
  sim.run(out, 1.5);
  assertThat(out.arrived()).isTrue();
  assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
  ```

- **The whole robot:** `RobotSim` runs it, drivetrain and gamepads included ([testing](testing.md)).

`src/test/java/frc/robot/examples/` has this guide's robot and its commands, compiled
(`ExampleRobot`, `ExampleCommands`, `Controls`): the guide's code is theirs, and a test fails when
the two drift apart. `src/test/java/frc/robot/robots2026/` has a real 2026 robot described this way. It runs end to end
in the simulator, gamepads, interlock, gates and auto included (`Team2611Controls` and
`Team2611RobotSimTest`).

The drivetrain has moves too: [driving](driving.md).

## 7. When the template can't drive it

The block says some mechanisms the template can't drive yet: a hood on linear servos, a
differential's pair. Some it can't say at all, such as an arm a motor swings through a linkage.
Either way, the joint stays in the definition, and a subsystem of your own drives it with WPILib's
API or its vendor's.

**Keep the joint in its block, and leave it undriven.** Write its shape, its range, its places,
what it carries and its sensors as for any joint, and its servos if it has them (`.servo(...)`), or
a differential's drives in the robot's definition. Leave out `.control(...)`: a control asks the
template to drive the joint, and one on a joint it can't drive stops the robot starting, saying
why. Motors in a joint's block are the template's to drive, so motors your own code drives stay out
of it.

```java
static final Position HOOD_DOWN = Position.degrees("down", 0);
static final Position HOOD_LOB = Position.degrees("lob", 30);

static final Actuator.Servo WCP_0408 =
    Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();

static final Joint.Revolute HOOD =
    Joint.pivot("hood", Part.CHASSIS, new Translation3d(-0.2, 0, 0.55), Joint.PITCH_UP)
        .range(Range.degrees(0, 40), HOOD_DOWN)
        .positions(HOOD_DOWN, HOOD_LOB)
        // Each servo's body on the chassis, from the hood's axle, and its rod's end on the hood.
        .servo(
            "hood left",
            WCP_0408,
            new Translation3d(0.054, 0.1, -0.233),
            new Translation3d(0.054, 0.1, 0))
        .servo(
            "hood right",
            WCP_0408,
            new Translation3d(0.054, -0.1, -0.233),
            new Translation3d(0.054, -0.1, 0)); // no .control(...): Hood drives it
```

Kept there, it's still the robot's own:
- **Checked** as every joint is: its places inside its range, each name once, its sensors' wiring
  against every other device's.
- **Its sensors are read** with the robot's: a beam on its link, or a switch at its end, is a gate,
  `sensors.gate("hood down", HOOD.switchAt(End.LOWER).tripped())`, though only your code stops the
  joint there.
- **Its places are its constants**, so `HOOD_LOB` means one thing in every file, and a practice
  robot changes the joint as it changes any (`replaced(...)`, `range(...)`).

What you give up until the template drives it:
- **Its moves and gates.** `joints.of(HOOD)` says why there's none ("hood has no mechanism: it's
  moved by hood left and hood right, and the template drives only motors turning a shaft so far"):
  no outcomes, no `isAt`, no `Joints/hood` in the log. Your subsystem's commands are plain v3
  commands, and it logs what you need.
- **Interlocks.** None may name it, as the joint held or the one read: nothing could enforce it,
  and the robot refuses it. A clash with it is your code's to prevent. A configuration that names
  it can't run, and the robot says so as it starts.
- **The simulator's plant.** It isn't simulated, and the template takes it as resting where its
  block rests it: a camera or a joint riding on it is placed from there. A switch on it is a test's
  to trip.

**Read the model; don't restate it.** Your subsystem takes the robot's definition and finds its
own joint there, so a practice robot's range and places are the ones it uses:
- the joint: `robot.joints().get(robot.joints().indexOf(HOOD))`;
- a place, by its constant: `hood.own(HOOD_LOB).value()`, in radians, or meters for one that
  slides;
- its hard stops: `hood.range()`;
- each servo's share of its travel at an angle, from where its block puts the servo: each is one of
  `hood.ownDrives()`, and `servo.command(math.coordinate(drive, Map.of(hood, angle)))` its share,
  `math` being `robot.math()`;
- an absolute encoder on a Systemcore port, in its block: its wiring,
  `hood.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port)`, names the port to read
  with WPILib's `DutyCycleEncoder`, and its `read(...)` turns what the port reads into the
  encoder's turns; `math.fromAbsolute(math.absoluteEncoder(hood), turns)` turns those into where
  the joint is. A CANcoder is read through its joint's motor controller, so a joint without motors
  in its block can't have one.

Only what the block can't say yet is written in the subsystem: for servos, their PWM ports and the
pulses their datasheet gives. WPILib 2027's alpha has no `Servo` class: a servo on one of
Systemcore's ports is a `PWM`, told a pulse in microseconds.

```java
final class Hood implements Mechanism {
  private final Scheduler scheduler;
  private final Joint hood;
  private final RobotMath math;
  private final List<Drive> servos;
  private final List<PWM> pulses;

  /** The angle its servos were last told, in radians: NaN until they're first told. */
  private double told = Double.NaN;

  Hood(Scheduler scheduler, RobotDefinition robot, PWM left, PWM right) {
    this.scheduler = scheduler;
    hood = robot.joints().get(robot.joints().indexOf(HOOD)); // this robot's own hood
    math = robot.math();
    servos = hood.ownDrives(); // its servos as its block names them: left, then right
    pulses = List.of(left, right);
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /** Goes to one of its places, and ends once its servos have had time to get there. */
  Command goTo(Position place) {
    return run(co -> co.wait(Seconds.of(tell(hood.own(place).value()))))
        .named("Hood to " + place.name());
  }

  // Tells each servo the share of its travel that puts the hood at an angle, held to its range,
  // and says how long the slower takes to get there. Nothing reads back where it is: it's assumed
  // where it was told before, or the first time, at the far end of its range.
  private double tell(double angle) {
    Range range = hood.range().orElseThrow();
    double to = Math.max(range.lower(), Math.min(range.upper(), angle));
    double far = to - range.lower() > range.upper() - to ? range.lower() : range.upper();
    double from = Double.isNaN(told) ? far : told;
    double seconds = 0;
    for (int i = 0; i < servos.size(); i++) {
      Drive drive = servos.get(i);
      Actuator.Servo servo = (Actuator.Servo) drive.actuator();
      double length = math.coordinate(drive, Map.of(hood, to)); // eye to eye, there
      // A WCP-0408 takes a pulse of 1000 µs fully retracted, to 2000 µs fully extended.
      pulses.get(i).setPulseTimeMicroseconds((int) Math.round(1000 + 1000 * servo.command(length)));
      double travel = Math.abs(length - math.coordinate(drive, Map.of(hood, from)));
      seconds = Math.max(seconds, travel / servo.speed());
    }
    told = to;
    Logger.recordOutput("Hood/Told", Math.toDegrees(to));
    return seconds;
  }
}
```

Build it in `Controls`, with the robot it runs on and its servos' ports, `new Hood(scheduler,
Robots.current().definition(), new PWM(0), new PWM(1))`, and bind its commands as any v3
mechanism's: `operator.faceUp().onTrue(hood.goTo(HOOD_LOB))`. `src/test/java/frc/robot/examples/`
has it compiled (`ServoHoodRobot`, `Hood`), and `HoodSimTest` checks the pulses it sends.

## Not yet

- **Joints the template doesn't drive yet:** a differential's pair, and servos. The robot says
  which, and why. An interlock on one is refused, since nothing could enforce it. A
  configuration naming one is refused whenever it would start, saying why, and the startup's
  doubts list it, so the rest of the robot still runs. Meanwhile, drive one from a subsystem of
  your own: [When the template can't drive it](#7-when-the-template-cant-drive-it).
- **A bench bypass for interlocks.** It waits for a pit mode.
- **Moving between configurations by a route,** such as first raising the arm so the elevator can
  drop. Until then, write the two steps.
