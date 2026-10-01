# Calibration

The robot's configuration starts from datasheets and CAD. These procedures replace those guesses
with the robot's own numbers. Each one runs from the robot's Pitwall page, works the number out,
and shows it on the page's **Calibration** line with the date and the exact line to put in the
robot's configuration. Nothing changes on its own: edit `Robots.java` (or the robot's wiring), and
redeploy.

Run them in this order, since each leans on the ones before it:

1. **The IMU reads level**, so tilt is measured from true level.
2. **The gyro's scale**, so every turn is measured right.
3. **The wheel radius**, measured against the gyro.
4. **Camera mounts**, measured while the other cameras place the robot.

Run every procedure in utility mode, enabled, with the robot on the floor. The page refuses to run
them on the field.

**Safety:** these drive the robot, spinning it in place or moving it between spots. Clear a circle
around it and keep everyone out of it. One named person at the Driver Station does nothing but
watch the robot, ready to disable it, and says "enabling" before it moves. Wear safety glasses, as
at events. ([Characterization's safety rules](characterization.md#safety-first) say more.)

Two more have nothing to run on the page: [the robot's mass, center of gravity, and
inertia](#the-robots-mass-center-of-gravity-and-inertia), measured by hand, and [a terrain
crossing](#a-terrain-crossing), read from a log. Put what they find in `Robots.java` the same way,
with the date: `.mass(Pounds.of(131.2)) // measured YYYY-MM-DD` ([docs/values.md](values.md) says
how every value is measured, and where each belongs).

## The IMU reads level

*Check the IMU reads level (2 s, on a level floor).*

Stand the robot still on a floor you trust to be level. The procedure averages the IMU's pitch and
roll for two seconds, and should read within 1° of zero. If it doesn't, the IMU's mount calibration
is off, and every tilt the robot reads is off by the same amount: tilt handling, bump detection,
and every camera image solved on a tilted robot. Redo the Pigeon 2's mount calibration in Phoenix
Tuner X, with the robot level, then check again.

When: after mounting or moving the IMU, and before each event.

## The gyro's scale

*Calibrate the gyro's scale (5 turns where cameras see tags).*

Park the robot where its cameras see at least two tags at once, and clear a circle around it. It
stands still for a second and a half while the cameras measure its heading, spins five turns in
place, stops, and measures again. The gyro counts the turns; the cameras say exactly where it
stopped. A gyro reading 0.3% high misreads five turns by 5.4°.

The result is a scale for the robot's wiring: `.gyroYawScale(0.99702)`. It already includes any
scale the wiring had, so replace the old value. If the cameras don't measure the heading at least
five times standing still, it says so: face more tags.

When: once for each gyro, and after replacing or remounting one.

## The wheel radius

*Calibrate the wheel radius (2 slow turns in place, 30 s).*

Clear a circle around the robot. It spins twice in place, slowly so the wheels roll instead of
scrubbing, and compares how far the gyro says it turned with how far the wheels turned: a wheel
that rolls around a circle of known size tells its own radius (6328's procedure). The report gives
the radius and each wheel's own; one wheel far from the rest is worn differently or slipping.

The result is a radius for the drivetrain: `.wheelRadius(Inches.of(1.974))`. Tread wears, and the
radius shrinks with it: measure before each event. Calibrate the gyro's scale first, since its error
carries straight into the radius.

## A camera's mount

*Calibrate the front-left camera's mount (20 s, drive slowly past tags).*

Each camera has its own action. Drive the robot to a spot where the camera sees tags, and stop for
two seconds; then another spot, nearer or farther, turned a little; six spots or so, from about a
meter to four meters away. The other cameras must see tags too: they hold the robot's pose while
the camera being calibrated is left out. Images count only while the robot stands still and its
pose is known within 5 cm. It stops after twenty seconds, or 600 sightings.

The procedure finds the mount that puts every tag the camera saw at its real place. The report says
how far the tags landed off (across the camera's line of sight, where it sees them sharply) before
and after, how sure it is, and the camera's mount as the builder calls to put in its entry in
`Robots`: `.forward(Inches.of(-7.61)).left(Inches.of(12.06)).up(Inches.of(16.97))
.roll(Degrees.of(0.0)).tiltUp(Degrees.of(10.0)).yaw(Degrees.of(23.2))`. If it says it isn't sure, the spots were too alike: a camera 4 in low and one tilted
down 2° put a tag 3 m away in the same wrong place, and only tags at different distances tell them
apart.

When: after mounting, bumping, or remounting a camera, and whenever the robot says a camera is out
of calibration.

### How the robot notices a camera that moved

The robot checks every camera's mount all the time, from the tags it sees while the other cameras
hold the robot's pose. Each tag, placed on the field through the robot's pose and the camera's
configured mount, should land on the tag. Noise puts tags off every which way and averages out; a
camera bumped since it was calibrated puts them off the same way every time. Over its latest 500
tags (seconds of seeing tags), the average of how far they land off is the Vision log's
`MountOffsetMeters` for that camera.

A tag counts only when the pose it's placed through can be trusted: the robot turning slowly, the
pose and heading sure enough that together they'd put a tag at that distance less than the
tolerance off, no calibration running, and another camera's measurement agreeing with the pose
within the tolerance in the last two seconds. A heading off by a degree puts a tag 3 m away 5 cm
off, the same way every time, which is just what a camera turned on its mount looks like.

Past 5 cm (the vision configuration's `mountToleranceMeters`, which the pit can tune), the alert
*Camera front-left is out of calibration: its tags land 10 cm off* says to recalibrate it. Until
then, the part of the offset across the floor (beyond the tolerance, taken off in squares) is how
much less the camera is trusted: it's added to each of its measurements' uncertainty
(`MountDoubtMeters` in the log), and the angle it makes at the tags' distance to its heading's. A
camera turned on its mount puts the robot off to the side, and its heading off by its turn; one
knocked straight down still places the robot right on the floor, so the alert fires but its trust
stays. The alert says "so it counts for less" when that's the case.

For example: a camera whose tags land 10 cm off across the floor, against the 5 cm tolerance, is
√(10² − 5²) = 8.7 cm less sure of every measurement. One of two tags 2 m away, which it would
place within 2 cm, it now places within √(2² + 8.7²) = 8.9 cm: it counts about a twentieth as
much. Its far tags, already uncertain, lose less. The wider uncertainty also loosens the gate the
estimator holds its measurements to, so they're rejected less often while they count for less.

Two patterns point somewhere other than the mount:

- **Tags that land farther off the farther away they are**, by a share of the distance (2 cm at
  1 m, 6 cm at 3 m), along the camera's line of sight: its lens calibration (focal length) is off.
  Recalibrate the lens in PhotonVision; recalibrating the mount only hides it at one distance.
- **Every camera out of calibration at once at an event:** the field's tags aren't where the
  layout says. Check the event's field layout (WPIcal) before touching any camera.

The check measures against the robot's pose, so it needs other cameras to hold it: a robot with
one camera can't tell that camera's mount moved. A wrong heading (the gyro's scale off, over many
turns) puts every camera's tags off as if each had turned, until the cameras correct the heading;
several cameras out of calibration at once more likely means that than several knocks. A badly
knocked camera also pulls the pose a little, so a camera beside it can read a few centimeters off
too: recalibrate the one farthest off first, and the other comes back with it.

## The robot's mass, center of gravity, and inertia

*By hand, with nothing to run.*

These set the robot's body: `Body.mass`, `Body.cgHeight`, and `Body.yawMomentOfInertia`. Measure
with the bumpers and battery on, the robot powered off. No Systemcore needed.

**Safety:** a robot weighs as much as a person. Lift it with two people, one at each end, knees
bent; chock every wheel on the scales and blocks, front and back, so nothing rolls; and keep hands
and feet out from under the robot, always: lift and lower it by the frame.

**Mass:** two bathroom scales, one under the front wheels and one under the back (a board across
each pair). The mass is the sum. The two readings also say how far forward the center of gravity
is.

**The center of gravity's height**, from CAD if the CAD is complete, or by tilting:

1. With the robot level, front wheels on a scale and the back on the floor at the same height (a
   board as thick as the scale), read the scale: `F₀`. Measure `L`, the distance between the front
   and back axles, and weigh the robot, `W`.
2. With two people lifting the back by the frame, a third slides a solid block, about 15 cm tall,
   under the back wheels, and they lower the robot onto it. It tilts by `θ`, where `sin θ` is the
   block's height over `L`. Chock the wheels again, and read the scale: `F`. Lift it off the same
   way.
3. The center of gravity's height above the axles is `L × (F - F₀) / (W × tan θ)`. Add the wheel
   radius for its height above the floor.

A bigger tilt gives a bigger change to read; check that nothing on the robot moves or hangs as it
tilts. With the tread's friction ([characterization](characterization.md#tread-friction)), the
height sets where the wheels slip as the robot accelerates, and so the drive's current limit.

**The yaw moment of inertia**: from CAD's mass properties, with the bumpers and battery
modeled; or from SysId's rotation routine (the wheels pointed around the circle, robot powered and
driving: [the safety rules](characterization.md#safety-first)), which fits the robot's angular kA
in volts per rad/s². Then `I = 4 × (stall torque / nominal voltage) × driveGearing × r /
(wheelRadius × kA)`, in kg m², with the drive motor's stall torque and nominal voltage from its
datasheet (or `Motor.model()`), and `r` each wheel's distance from the robot's center. Friction
makes it read a little high. Nothing in the code reads it yet; a simulator that models traction
will.

When: once the robot is built, and after a big change.

## A terrain crossing

*From a log of driving over a bump, with nothing to run.*

How the robot's IMU reads crossing a bump or a ramp: how far past the ramp's slope it tilts
(bumpers, suspension, the IMU's mount), and how closely the moment the IMU says it crossed the
crest matches where the robot was. It checks the terrain thresholds (`TerrainCrossing.Settings`)
and the tilts at which the estimator trusts the wheels less (`EstimatorConfig.tiltStartDegrees`
and `tiltZeroDegrees`), and sets the slope margin and the crest's spread in `Localization` (3° and
5 cm today).

You need the robot running its program on its Systemcore, and a bump or ramp built to the field's
drawings, on field carpet, with tags in the cameras' view of it.

**Safety:** it drives at match speed over a bump: the driving rules above, with nobody near the
bump, since a robot crossing at an angle can tip.

1. Drive over it straight across at a crawl, at a medium speed, and at match speed, three times
   each, logging every run.
2. Again at 30° and at 60° from straight across, and backward.

Then, from the logs:

- **The slope margin:** the most `Localization/TiltDegrees` read on the bump, less the ramp's
  slope. The margin should be above it.
- **The thresholds:** in each run, `Localization/ClimbDegrees` should pass `steepDegrees` going up
  and coming down, `Localization/Terrain` should go from climbing to descending to level, and
  `Localization/TerrainCrossings` should count each crossing once, and never in flat driving.
- **The crest's spread:** each crossing logs the crest time (`Localization/CrestSeconds`). Where was
  the robot's center then, across the crest? To see it from the cameras alone, replay the log with
  the crest's spread in `Localization` set far larger (a meter), so the cameras place the robot and
  the crest barely moves it. The standard deviation of the robot's distances from the crest line,
  at the crest times, is the spread.

Every crossing should be counted and none in flat driving, the tilt past the slope within a few
degrees, and the crest within a few centimeters at every speed. If fast crossings read differently
from slow ones, the robot's bumpers or suspension are moving the IMU more than the ground is.

When: once the robot's bumpers, suspension, and IMU mount are final, and at kickoff, when the new
field's bumps and ramps are known.

## Done elsewhere

- **Camera lenses** (intrinsics): in PhotonVision, with a ChArUco board, at the resolution the
  robot runs. PhotonVision keeps the result; export it with the robot's settings.
- **The field** (tag positions at an event): WPILib's WPIcal, from photos of the field. The result
  is a field layout for that event.
