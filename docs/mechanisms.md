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
what it carries, the places it goes, its motors and their wiring, its own sensors, and how the
program drives it. The places come first, as constants, so your code names them, and a misspelled
one won't compile.

```java
static final Position PIVOT_IN = Position.degrees("in", 1.5);
static final Position PIVOT_OUT = Position.degrees("out", 130);

static final Joint.Revolute PIVOT =
    Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
        .range(Range.degrees(0, 135), PIVOT_IN)          // its hard stops, and where it rests
        .carries(Link.box("intake arm", new Translation3d(0, 0, 0.2),
            new Translation3d(0.05, 0.55, 0.36)))
        .positions(PIVOT_IN, PIVOT_OUT)
        .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32))
        .absoluteEncoder(PIVOT_CANCODER)
        .control(MechanismControl.position(30).withProfile(8, 30));

static final Output INTAKING = Output.rpm("intaking", 2000);
static final Output OUTTAKING = Output.volts("outtaking", -4);

static final Joint.Revolute ROLLER =
    Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))   // it rides on the pivot's end
        .outputs(INTAKING, OUTTAKING)
        .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31))
        .control(MechanismControl.velocity(0.02));
```

Then list the joints in the robot's definition: `.joints(List.of(PIVOT, ROLLER))`.

- **`Joint.pivot`, `Joint.roller` and `Joint.elevator`** start the common joints; each call after
  adds one thing. For anything else, the builders (`Joint.Revolute.builder()`,
  `Joint.Prismatic.builder()`) take every value by name.
- **A link** is what the joint moves. `Link.box` draws it as a box for the extension checks, its
  mass unknown; `Link.weighed` adds a mass from CAD or a scale; `Link.unseen` is neither. A mass
  is optional, and a rough one does: the simulator reads it, and beyond that it only seeds a
  joint's kG and kA until you tune them ([Gravity](#gravity)). One left out is never guessed: the
  simulator says what it does without it.
- **Positions** are named places a joint goes: "in", "out". They're relative to the joint's
  parent, so an arm's "level" means the same at any elevator height.
- **Outputs** are named ways a joint runs: a speed (`Output.rpm`), a voltage (`Output.volts`) or a
  share of the battery (`Output.dutyCycle`). Every joint also has `Output.STOPPED`. Two joints
  that both have a "feeding" each get their own constant: `FEEDER_FEEDING`, `INDEXER_FEEDING`.
- **Motors** turn it through its gearing (`.motor(...)`, or `.motors(...)` for several together,
  the leader first), and onto a drum for one that slides (`.motors(Motor.KRAKEN_X60, gearing,
  drumRadius, ...)`). With no controllers named, it runs in the simulator alone. A drive that
  turns two joints at once, a differential's, is the robot's own (`.drives(...)` in the
  definition).
- **Its own sensors**: an absolute encoder turning with it (`.absoluteEncoder(...)`, see
  [Absolute encoders](#absolute-encoders)), and the limit switches it presses at the ends of its
  travel (`.limitSwitch(End.LOWER, Degrees.of(0), ...)`). [Sensors](#sensors) has the rest.

**How it's driven isn't part of what the joint is.** A joint re-tuned or re-wired is the same
joint: everything that names it still finds it. So a practice robot of the same design is the
competition robot with its differences, and the same code runs on either:

```java
static final RobotDefinition PRACTICE =
    COMPETITION.with(PIVOT.motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(33)));
```

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

  ```java
  static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, point(0.36, 0, 0))
          .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31))
          // The roller runs only once the pivot is out of the frame.
          .runsOnlyWhen(PIVOT.past(Degrees.of(110)))
          .control(MechanismControl.velocity(0.1));

  static final Joint.Revolute ARM =
      Joint.pivot("arm", ELEVATOR, point(0.1, 0, 0.3), Joint.PITCH_UP)
          ...
          // The arm keeps out of the bumper's band while the elevator is too low to clear it.
          .keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.before(Inches.of(32)))
          .control(...);
  ```

  An edge is usually a clearance, a number from CAD or measured where the parts just clear, not a
  place the joint goes, so it's written as one: `PIVOT.past(Degrees.of(110))` means 110° lies
  between the pivot's reference and where it is now, whichever way it's moving. `before` and
  `between` complete the set, and a named position works wherever the edge really is one
  (`ELEVATOR.before(ELEVATOR_SCORE)`). `SHOOTER.atLeast(RPM.of(3000))` holds a joint until another
  is up to speed. A sensor's conditions work as well (`.runsOnlyWhen(CLAW_BEAM.clear())`), and a lost
  sensor restricts: a joint kept out stays out, one that runs only when it holds is held.

  A block can only name joints written above it, so a joint restricted by one that rides on it (an
  elevator held up while its own arm is down) has its rule in the robot's definition instead:
  `.interlocks(List.of(Interlock.that(ELEVATOR).keepsOut(...).when(ARM.before(...))))`, written
  the same way. Either place is fine; both are enforced and checked together, and a rule written in
  both is refused. The definition checks interlocks on joints against each other and the
  configurations; one on a sensor, or on a speed, it can only check is the robot's, since neither is
  known until the robot runs.

**The definition checks itself** when the robot starts, and in the test that builds every robot,
so a mistake fails the build, not a match. It refuses:
- a position outside its joint's range, or a name used twice;
- a position, an output or a configuration's place that isn't that joint's own;
- a speed faster than the joint's motors can turn it, through its gearing (it notes one within 10%
  of it, since the motor can't hold that under load);
- a configuration that breaks an interlock, two joints whose interlocks wait on each other, or an
  interlock written twice.

## 2. How it's driven and wired

### Control

`.control(...)` says how the program drives the joint: gains, soft limits, motion profile, current
limits, neutral mode, bands and homing. Every value is in the joint's own units, radians or meters.
`MechanismControl.position(kP)` and `velocity(kP)` start from the defaults (the feedforward from
the definition, 40 A, braking), and each `with...` changes one value; a flywheel adds
`.coasting()`. A joint with motors needs one: the robot won't start without it, and says what to
give.

[values.md](values.md) says what each value means and how to find yours. The robot refuses to
start if a named position lies outside its soft limits, where it would be sent and never get.

Its motor controller's settings are worked out from its control as the robot's configuration is
checked: when the robot starts, in the simulator, and in the test that builds every robot. What the
controller can't take is refused there, in the words the robot would use: a gravity angle past
Phoenix's quarter turn, a profile cruising faster than the motors turn the joint, a ratio or a gain
past what Phoenix holds, a kI on a SPARK. They're worked out for the controller the joint is wired
to, and for a joint with no wiring, for a TalonFX, which is what the simulator runs it as. What's
doubtful, though the controller takes it (a tuned kG far from what the joint's weight needs, a
gravity offset at Phoenix's limit), is noted under `Joints/<label>/SettingsNotes` when the joint
starts, in the simulator as on the robot, and on the robot's Driver Station console.

- **Gains.** kV, kA and kG start from the definition, worked out from the motors, gearing and mass,
  until you measure them ([characterization](characterization.md)). kP, kI and kD are yours. kG
  is the one that matters most for a joint held against gravity: [Gravity](#gravity).
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
  | none (empty) | The joint has an absolute encoder in its block. Nothing to do: it's told where it is at startup, from a CANcoder or a SPARK's data port by its motor controller, or from a Systemcore port by the robot (below). |
  | `Homing.toHardStop(Stop.LOWER)` | No absolute encoder. On the first enable, the joint is pushed gently (2 V) into that hard stop, and its position there becomes the stop's. Moves wait for it; if it finds no stop in time (below), the joint is faulted and its moves are refused. Check on the robot that 2 V really moves it. |
  | `Homing.toSwitch(Stop.LOWER)` | No absolute encoder, and a limit switch at that end whose position its block knows. On the first enable, the joint is driven gently (2 V) toward it, and its position as the switch trips becomes the switch's; one starting on it backs off first. It's as close as half the joint's travel in a loop at that speed: slower is closer. Moves wait for it; if the switch doesn't trip in time (below), the joint is faulted, and says if the switch is lost. |
  | `Homing.assumeAtReference()` | No absolute encoder, and you stow the joint by hand before every power-on. It trusts that. |

  A joint held at positions without an absolute encoder must name one of these: the robot won't
  start until you choose.

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

```java
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

A joint with no wiring runs in the simulator, its control checked as a TalonFX would take it. On a
robot it stays still, and an alert says so.

**On REV's SPARK MAX and SPARK Flex**, a joint is driven the same way, from the same control: its
SPARK's settings come from the ones a TalonFX would get. A few things differ:
- The SPARK's loop runs kP; the feedforward, gravity and kD are worked out on the robot each loop
  and sent with it as volts.
- A SPARK limits its motor's current (from the control's stator limit), not the battery's.
- An absolute encoder goes on the SPARK's data port (`.absoluteEncoder(dataPortEncoder)`), read
  once at startup, or the SPARK's loop closed on it ([Absolute encoders](#absolute-encoders)); a
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
| A CANcoder | `.absoluteEncoder(cancoder)` | `Feedback.ROTOR`: the robot reads it at startup and tells the joint's TalonFX | `REMOTE`, `FUSED` or `SYNC`: the TalonFX's own loop, at 1 kHz |
| A SPARK's data port, such as REV's Through Bore | `.absoluteEncoder(dataPortEncoder)` | The SPARK, read at startup | `dataPortEncoder.closesLoop()`: the SPARK's own loop, at the controller's rate |
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
simulator a joint on a SPARK runs as on a TalonFX, its loop on its motor's count, so a slip the
encoder would correct isn't simulated.

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
of the joint's: `.absoluteEncoder(AbsoluteEncoder.of().ratio(0.536).on(cancoder))`. Where the joint
is, is known within one of the encoder's turns; the robot says if that window doesn't cover the
joint's travel.

**Counts per turn** are optional. Only a pair of encoders on one joint geared nearly alike reads
them, a turret's on 17 and 18 teeth decoded by their difference, and the robot refuses that pair
without each one's, from its datasheet: `.absoluteEncoder(4096, cancoder)`, or
`AbsoluteEncoder.of(4096)`.

**Offsets.** Each encoder's offset makes it read zero with the joint at zero, the pose its origin
gives in CAD. A team's code often holds one already: a CANcoder's `MagnetOffset` is its offset as
it is. A SPARK's `zeroOffset` is the raw reading at zero, before inverting, so it's the negative
here, `-zeroOffset`, or `+zeroOffset` with `reversed(true)`; from it, the template tells the SPARK
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
SensorWiring.Direction.REVERSE, NORMALLY_CLOSED)` on its own controller's limit pin, or with a
Systemcore port or a CANdi's input in place of the pin. Each is named for its joint and end,
"pivot lower limit", and `PIVOT.switchAt(End.LOWER)` is the switch, for a condition. One whose
position isn't known (`.limitSwitch(End.LOWER)` without an angle) can stop its joint but not home
it.

**Every other switch and rangefinder** is a constant of its own, on its part, the chassis or a
joint's link (the arm a roller rides on, not the roller, which spins), carrying its wiring:

```java
static final Sensor.Switch CLAW_BEAM =
    Sensor.Switch.beam("claw beam", PIVOT)
        .on(new SensorWiring.Candi(CAN, 30, 1), SensorWiring.Polarity.NORMALLY_OPEN);
static final Sensor.Rangefinder HOPPER = ... .on(new SensorWiring.CanRange(CAN, 31));
```

Add them to the definition (`.sensors(...)`). A switch is tripped or clear, whatever its wiring;
its kind says what trips it: a `beam` break, or a `contact` switch nothing in the model closes (a
button, a cage sensor). A rangefinder reads a distance along its axis. Where on its part it sits is
optional: a team rarely knows a beam's exact points, and the missing-data report says what's a
guess. What one means is a condition over it, either way: `CLAW_BEAM.tripped()`,
`CLAW_BEAM.clear()`, a rangefinder's `nearerThan(Inches.of(6))`.

A switch's wiring is where it's connected (a Systemcore port, a CANdi's input, or a limit input on
a joint's motor controller: a SPARK's, a TalonFXS's or a Falcon's; a Kraken has none) and its
polarity, normally open or normally closed, which nothing above the wiring sees. Wire a limit
switch normally closed: a cut wire then reads tripped, and its joint stops. A sensor with no wiring
runs in the simulator alone. The robot refuses two sensors on one input, a controller's limit input
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
lower limit reads tripped with pivot at 130.0°, nowhere near where it trips, 30.0°: a cut wire, if
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
| `REFUSED` | It wouldn't start: a target outside the limits, a joint that isn't answering or failed to home, or its `onlyIf` gate false |

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

Write what the robot does in one class: its intake, its shot, its autos. Build it in
`RobotContainer`, after the joints, and bind it there:

```java
final class Actions {
  private final JointMechanism pivot, roller;

  Actions(Joints joints) {
    pivot = joints.of(PIVOT);
    roller = joints.of(ROLLER);
  }

  /** Deploy and roll. The interlock holds the roller until the pivot passes 110°. */
  Command intake() {
    return Command.requiring(pivot, roller).executing(co -> {
      co.fork(pivot.goTo(PIVOT_OUT));
      co.await(roller.run(INTAKING));
    }).named("Intake");
  }
}

// In RobotContainer:
operator.rightTrigger().whileTrue(actions.intake());
operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));
```

Each driven joint is its own v3 `Mechanism`, so a command requires exactly the joints it moves.
Deploying while the shooter spins is fine, and one button can't cancel another's unrelated work.

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

```java
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

```java
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
| `Outcome` | How its last move ended: "pivot → out: TIMED_OUT after 1.60 s, pivot at 40.0°, held: …" |

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

`src/test/java/frc/robot/robots2026/` has a real 2026 robot described this way. It runs end to end
in the simulator, gamepads, interlock, gates and auto included (`Team2611Actions` and
`Team2611RobotSimTest`).

The drivetrain has moves too: [driving](driving.md).

## Not yet

- **Joints the template doesn't drive yet:** a differential's pair, and servos. The robot says
  which, and why. An interlock on one is refused, since nothing could enforce it. A
  configuration naming one is refused whenever it would start, saying why, and the startup's
  doubts list it, so the rest of the robot still runs.
- **A bench bypass for interlocks.** It waits for a pit mode.
- **Moving between configurations by a route,** such as first raising the arm so the elevator can
  drop. Until then, write the two steps.
