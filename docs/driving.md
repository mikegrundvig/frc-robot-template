# Driving

The drivetrain has moves, as a joint does ([mechanisms](mechanisms.md#3-use-it)). Each move is a
Commands v3 command that ends on its own, with a deadline, an outcome (`ARRIVED`, `TIMED_OUT`,
`CANCELED` or `REFUSED`) and a reason, logged as `Drive/Outcome`. For example: "drive → (2.00,
1.00) 90.0°: ARRIVED in 2.04 s".

| Move | Does | Ends |
|---|---|---|
| `drive.runUntil(new ChassisVelocities(1.5, 0, 0), bumped)` | Drives at a robot-relative velocity until a condition holds | When the condition holds; after 3 s otherwise, saying why a gate never opened |
| `drive.turnTo(Rotation2d.fromDegrees(90))` | Turns where it stands to a field heading, the nearer way round | Within 2°, nearly still, for 40 ms |
| `drive.goTo(new Pose2d(2, 1, Rotation2d.fromDegrees(90)))` | Drives to a field pose in a straight line, turning on the way | Within 2 cm and 2°, nearly still, for 40 ms |
| `drive.follow(keyframe, keyframe, ...)` | Drives a smooth path through keyframes, stopping at the last | As `goTo`, at the last keyframe |
| `drive.goTo(pose).within(Seconds.of(4))` | The same move with another deadline | As above |

A move that drives a path has a default deadline: twice the path's time, plus half a second. A
move that times out says where the robot got to: "TIMED_OUT after 0.50 s, at (0.30, 0.00) facing
0.0°".

`src/test/java/frc/robot/examples/Explorer.java` shows the moves in use. Its robot drives forward
until its bumper switch trips, then turns a quarter turn either way, and repeats:

```java
Command explore() {
  return Command.requiring(drive).executing(co -> {
    while (true) {
      DriveMove forward = drive.runUntil(FORWARD, bumped).within(Seconds.of(15));
      co.await(forward);
      if (!forward.arrived()) {
        return; // the bumper never tripped: the move says why, and the robot stops
      }
      co.await(drive.turnTo(nextQuarter()));
    }
  }).named("Explore");
}
```

## How a move drives

A move plans its path when it starts, from where the robot is and how fast it's moving then. The
limits come from the robot's own model ([values](values.md)):

- **Linear acceleration.** Each wheel pushes with the lesser of two forces: what its motor gives at
  the stator current limit, through the gearing, and what its tread grips on its share of the
  robot's weight.
- **Turning acceleration.** The same wheel forces, about the robot's yaw inertia.

A move plans at half of these accelerations. The yaw inertia is an estimate until the robot is
characterized, and the simulator turns the robot more sluggishly than the model says.

Each loop, the move works out what to ask of the drivetrain:

- **Speed and turning rate:** what the path asks for at the end of the loop, plus:
  - a push back toward the path's place (`translationKp`);
  - a push back toward the path's heading (`headingKp`);
  - a push toward the path's speed.
- **Each wheel's volts:** each wheel also gets the volts its share of the path's acceleration takes.
  That way the robot keeps up with the path, rather than trailing it.

Both gains are in [values](values.md#drivetraincontrolgains). In the simulator, a quarter turn
planned at 0.83 s arrives in 0.88 s, and a path through three keyframes passes each within 2 cm
and 2°.

## Keyframes

A keyframe is a field pose the path passes through. The robot turns to face the keyframe's
heading on the way:

```java
DriveMove leg = drive.follow(
    Keyframe.at(new Pose2d(3, 1, Rotation2d.ZERO)),
    Keyframe.at(new Pose2d(5, 2, Rotation2d.CCW_90DEG)).named("pickup"),
    Keyframe.at(new Pose2d(7, 2, Rotation2d.PI)));
```

- **The path's shape** is a smooth curve through the keyframes' places: WPILib's cubic spline. It
  leaves toward the first keyframe, and arrives from the one before the last.
- **The path's speed** is kept under the top speed, and under the acceleration both along the path
  and across it in curves.
- **The robot's heading:** once the robot has passed a keyframe and faced its heading, it turns
  toward the next keyframe's heading, as fast as its turning limits allow, the nearer way round. If
  it hasn't finished turning to the last heading when it reaches the last place, it turns there.
- **Refused:** a keyframe within 5 cm of where the robot starts, or of the keyframe before it. No
  smooth path goes through those.

### Do something at a keyframe

A named keyframe is something code can wait for. `leg.at("pickup")` is a
[gate](mechanisms.md#4-gates-wait-for-things-and-know-why-not): it opens when the path passes the
keyframe, and stays open until the move starts again. `src/test/java/frc/robot/examples/PickUp.java`
has it whole:

```java fragment
Command pickUp() {
  return Command.requiring(drive, pivot).executing(co -> {
    DriveMove leg = drive.follow(...);
    co.fork(leg);                       // drive the path...
    co.waitUntil(leg.at("pickup"));     // ...and as it passes pickup,
    co.await(pivot.goTo(PIVOT_OUT));    // swing the intake out
    co.await(leg);                      // then wait for the path's end
  }).named("Pick up");
}
```

The gate opens on the path's timing, not on the robot's position. The robot follows the path to
within a few centimeters, so in the simulator the gate opens within 7 cm of the keyframe.

## Not yet

- **Paths from a file,** such as PathPlanner's or Choreo's. The path tool is each team's choice
  ([stand-ins](stand-ins.md)). A planned path is a WPILib `HolonomicTrajectory`, so a path read
  from a file could be followed the same way.
- **Obstacles.** A path goes where its keyframes put it. Nothing steers it around anything.
- **The robot's real limits.** Moves plan at half the model's accelerations until the robot's
  yaw inertia is measured ([calibration](calibration.md#the-robots-mass-center-of-gravity-and-inertia)).
