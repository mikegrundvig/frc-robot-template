# The estimator study

`./gradlew estimatorStudy` sets the robot's own pose estimator beside WPILib's
`SwerveDrivePoseEstimator` on the same simulated robot, in the same loop, and scores each against
the simulator's true pose. It answers one question: is the robot's estimator better than the
built-in one, by how much, and where is it worse?

It is a study, not a test: it runs outside `ci`, and no test demands that either estimator wins.
The one test (`EstimatorStudyTest`) checks that the study runs and that every estimator agrees with
the truth when the wheels and IMUs are exact, which is what catches a wrong unit, clock, or reset
in how WPILib's estimator is fed.

```
./gradlew estimatorStudy                                         every scenario, 20 seeds
./gradlew estimatorStudy --args="--scenarios=clean,shove --seeds=50"
./gradlew estimatorStudy --args="--list"                         the scenarios, and the field's tags
./gradlew estimatorStudy --args="--trace --cost"                 loop-by-loop traces, and each estimator's cost
./gradlew estimatorStudy --args="--holdout --seed=101"           the held-out set, on seeds of its own
./gradlew estimatorStudy --args="--cameras=pessimistic"          the same, with worse cameras (below)
```

Every scenario at 20 seeds is 820 simulated robots, more than one program's memory holds under a
2 GB heap: run it a few scenarios at a time, each to its own `--out`, and put the results together
with `--args="--merge=DIR,DIR,... --out=DIR"`.

**Worse cameras.** The simulated cameras scatter a tag's corners about a quarter pixel, well under
what a real camera may, and sit exactly on their configured mounts. `--cameras=pessimistic` asks
whether a verdict hangs on that: every scenario's cameras then scatter corners 0.6 px on average
(the simulator's spread in proportion), detect tags only to 5 m, and each sits off its configured
mount by a draw, fixed for the run, evenly within 5 mm along and 1° about each axis. These are the
worst ends of the ranges the camera placement study draws its robustness trials from
([runtime/docs/camera-placement.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/camera-placement.md#robustness)). A seed draws
the errors once, so every estimator sees the same images, and they are told the configured mounts,
as on a real robot. The draw adds to a scenario's own mount error (the `-mounts` scenarios'), and a
camera knocked or turned during a run moves from where it sits. Nothing is re-tuned: every estimator
keeps its settings. The default, `--cameras=nominal`, leaves every world as it was.

Results go to `build/studies/estimators/`: `summary.md` (the verdicts and a table for each
scenario), `summary.csv` (every measure's median and quartiles), `runs.csv` (every run),
`fed.csv` (how many measurements each estimator was offered and took, and how many corrections
WPILib's discarded for an older measurement), and with the options above
`trace-<scenario>.csv` and `cost.csv`. The same options give the same numbers: the simulation
repeats exactly by seed.

## The contenders

All of them see the same robot in the same run. After each robot loop the study hands WPILib's
estimators what that loop's sensors read.

Every contender is a localizer, and the study reaches it only through the runtime's `Localizer`
interface ([runtime/docs/localizers.md](https://github.com/mikegrundvig/frc-robot-runtime/blob/main/docs/localizers.md)): A is the robot's own, run
by the robot, and the rest are `WpilibLocalizer`, WPILib's estimator as a robot would choose it.
Each is given the loop's readings, then a placement if there was one, then its measurements, as
the contract has it. So the study exercises exactly what a team's own localizer would be given.

| | What it is | Its wheels and yaw | Its cameras |
|---|---|---|---|
| **A** | The robot's estimator, as the robot runs it | Every high-rate wheel reading, the gyro (or the backup IMU, or the wheels) | Every image, through the robot's vision pipeline |
| **B** | WPILib's, same inputs | The same wheel readings, one `updateWithTime` each, with the yaw below | Every measurement A's pipeline handed A's estimator, after the pipeline's filtering, with the pipeline's standard deviations |
| **C** | WPILib's, as a strong team runs it | As B | Every image, solved and filtered by the rules below |
| **D** | WPILib's, raw | As B | Every image, solved, unfiltered, with WPILib's default standard deviations (state 0.1, vision 0.9) |
| **O** | WPILib's odometry alone | As B | None: what the cameras are worth |

**The yaw** WPILib's estimators get is what `WpilibLocalizer` works out, as a robot with a backup
IMU would give them: the gyro's
at each wheel reading while it reports, the backup IMU's while only that does, and the wheels' own
turning with neither, carried on across a change of source so two IMUs' zeros never show as a turn.

**The order.** B's measurements reach WPILib's estimator oldest first within a loop, which is
`WpilibLocalizer`'s doing: WPILib discards its later corrections when it is handed an older
measurement.

**The reset.** When the robot is placed on its start, A is told so and WPILib's estimators get
`resetPosition` to the same pose in the same loop. A is also told how sure the placement is; WPILib's
reset has nowhere to put that.

**Solving an image** for C and D is what PhotonVision's own pose estimator does: the coprocessor's
multi-tag solution when it made one, otherwise the least ambiguous tag's likelier pose.

### C's rules, and where each comes from

Each is a rule found in strong teams' public robot code, with that code's constant.

| Rule | Value | From |
|---|---|---|
| One tag: reject above an ambiguity | 0.3 | The widely used open-source swerve template |
| Reject a solve off the floor by more than | 0.75 m | The same template |
| Reject a solve outside the field | no margin | The same template |
| Standard deviation, x and y | 0.02 m × distance² / tags | The same template |
| Standard deviation, heading | 0.06 rad × distance² / tags; one tag's heading ignored | The same template |
| State standard deviations | 0.1 m, 0.1 m, 0.1 rad (WPILib's defaults) | The same template, and two top teams that use WPILib's estimator |
| One tag: reject farther than | 3 m | A top team's variant of the template |
| Any image: reject farther than | 6 m | A top team that uses WPILib's estimator |
| Reject while the robot turned faster than | 5 rad/s in the last 0.3 s | A top team |
| Reject while the robot pitches or rolls more than | 5° | A top team |
| Wheels and yaw at the high rate | every reading, `updateWithTime` | The template, and most teams that use WPILib's estimator |

C's tunings, each run beside it, so that "A wins" can't be an untuned baseline:

| Name | What differs |
|---|---|
| `C.03` | State standard deviations 0.03 m and 1° (a top team's) |
| `C.01` | 0.01 m and 0.002 rad (a top team's) |
| `C.003` | 0.003 m and 0.002 rad (a top team's) |
| `C.003n2` | The same, with 0.01 m and 0.03 rad × distance² / tags² (that team's noise model) |
| `C.03sorted` | `C.03`, a loop's images added oldest first across the cameras |
| `C.03strict` | `C.03`, ambiguity at most 0.2 and height within 0.2 m (two top teams') |
| `C.03pick` | `C.03`, one tag's pose picked by the estimate's heading (two top teams') |
| `C.03gyro` | `C.03`, the heading left to the gyro alone |

B runs on three state standard deviations too (`B`, `B.03`, `B.003`). The report lets one tuning
stand for B and one for C: the one with the lowest position error over the clean scenarios, chosen
once and held for every scenario, as a team would tune on ordinary driving.

## The runs

A route is driven as a driver drives it: the sticks steer for each waypoint by where the robot
really is. Each run starts with the robot placed on its start, a second disabled while its cameras
see it, then enabled. The routes are written against the 2026 field: `short` (about 36 s: sprints,
a strafe while turning, a stop a meter before the hub's tags, a trench, a stop in the middle of the
field), `bumps` (over the bump and back, twice), and `match` (2 minutes 30 seconds, five cycles).

The faults are the simulator's own (`FaultCatalog`), applied through its controls as the Pitwall
page applies them. Two are the study's: a camera that reads a tag as another tag, and one that takes
a tag's two poses the wrong way round. Both change what the camera sends, so A's pipeline and every
other estimator get the same wrong image.

Five scenarios check whether a verdict hangs on the simulator's own idea of error. They drive the
clean match with each camera a little off its mount and each tag a little off the layout (errors
that repeat, and that cameras share), with those doubled, with the tags alone off, with the mounts
alone off, and with 2% of every camera's images flipped (errors with long tails).

## The held-out set

The scenarios above are the tuning set: the estimator was changed while looking at them. A second
set, behind `--holdout`, was written before any of those changes. It is looked at once on the
unchanged estimator, and then once after each round of changes is finished, with nothing adjusted
afterwards; it is also run whenever the robot or the field changes. Its 19 scenarios are
variations the tuning set does not have:

- the gyro's scale off by 1%, by 6%, and low instead of high, and a drift of 30° a minute;
- a camera turned on its mount by 1.5° and by 6° in yaw, and by 3° in pitch;
- a camera's clock off by 10 ms and by 40 ms;
- two faults at once: the gyro's scale with a knocked camera, and a clock off on low grip;
- cameras that all read their tags' range a little long, and cameras whose noise is shared;
- two cameras in place of four, a faster route, and a route far from the tags;
- the clean match with the mounts off, the tags off, and 5% of images flipped.

It runs on seeds 101 to 120, which no tuning run used. A result on the held-out set says more than
one on the tuning set, and the results below say which is which.

## The measures

For each estimator in each run, against the true pose, over every loop from the first:

- the position error's RMS, 95th percentile, and worst, and the heading error's;
- the same over the 2 s after each fault;
- how long after a fault the estimate is back within 5 cm and 1° and stays there 0.2 s ("never" if
  not within the run);
- the largest jump: how far the estimate moved in one loop beyond how far the robot did (what a
  driver, or an auto-align, feels);
- the error at rest, over the last half second of each stop near a tag and far from tags.

For A alone, which reports its uncertainty, the study also counts the share of loops where the
true error was past the estimate's own 95% bound, for position and for heading. An honest estimate
is past it in about 5% of loops. This is how the study checks that an estimate has not grown surer
than it is.

The report gives each as its median across the seeds with the interquartile range. A verdict of
"same" means two medians differ by less than the wider interquartile range. The seeds change only
the sensors' noise, so that range is narrow: read the size of a difference, not only its verdict.

`--cost` plays one run's inputs again into a fresh estimator of each kind, alone on the thread, and
gives each loop's time and the bytes it allocated. A is timed from its drivetrain readings to its
update, given its pipeline's measurements; B the same; C and D with solving their images.

## What it found

These are the results of the study as it stands, 20 seeds a scenario. Ours is A. WPILib's is
`C.003n2`, the tuning of C chosen before the comparison. Five measures are compared in each
scenario: the position error's RMS, the heading error's RMS, the error at rest near a tag, the
largest jump, and the time to recover. Two medians count as the same when they are within 1 cm
and 0.2°, or within the seeds' spread where that is wider.

The robot is the example robot: a camera at each corner of its frame, facing 45° out, so between
them the four look all the way round. The field is the 2026 field with its hub, trenches, towers
and outposts standing, which hide tags from the cameras.

| | Ours ahead | The same | WPILib's ahead |
|---|---|---|---|
| Tuning set, 41 scenarios, 195 comparisons | 33 | 153 | 9 |
| Held-out set, 19 scenarios, 87 comparisons | 11 | 70 | 6 |
| Worse cameras (`--cameras=pessimistic`), tuning set | 58 | 129 | 8 |
| Worse cameras, held-out set | 19 | 65 | 3 |

Earlier runs counted otherwise, and each was a different experiment:

| Run | Tuning set | Held-out set |
|---|---|---|
| Cameras in the rear half of the frame, an empty field, before the estimator corrected its gyro and watched its cameras | 44, 128, 23 | 10, 52, 25 |
| The same robot and field, after it did | 50, 136, 9 | 17, 69, 1 |
| Those cameras, on the field as it stands now | 47, 138, 10 | 18, 65, 4 |
| The cameras at the corners, on the field as it stands now | 32, 147, 16 | 10, 66, 11 |
| The same, with a recovery that needs independent evidence (below) | 33, 149, 13 | 10, 66, 11 |
| The same, with worse cameras | 30, 155, 10 | 12, 64, 11 |
| The same, learning each camera's turn on its mount (below), the cameras as the simulator has them | 33, 153, 9 | 11, 70, 6 |
| And with worse cameras | 58, 129, 8 | 19, 65, 3 |

The rear-half cameras are two facing forward and two straight back. The third row was run to
split the change between the second and fourth, when both the cameras and the field changed at
once: nearly all of it came with the cameras, little with the field. Of the 33 tuning-set
verdicts that changed between those two runs, 30 changed with the cameras, 2 with the field, and
one with both; of the 15 on the held-out set, 10, 3 and 2.

Much of what changed with the cameras is WPILib's gaining. With cameras that see tags from nearly
everywhere, its estimator behind its filter improved in most scenarios (a clean match from 1.4 cm
to 0.8, a frozen camera from 13.7 cm to 0.7), while ours stayed about where it was in clean
driving and lost ground in a few faults.

**Each camera's turn on its mount, learned.** The estimator now keeps each camera's turn on its
mount in its state and learns it as the robot drives (the localization README says how). The
turn starts as unsure as the camera's mount tolerance in the robot's description (left out, 5 mm
and 1°, a mount measured from CAD and checked). The gyro tells the cameras' turns apart from one
another; what they share is carried, never corrected, since only the wheels could tell it from a
heading error and a wheel's own steady error would be learned in its place. A camera whose
headings turn all at once has its turn opened up and learned anew. Before, each turn was taken as
known to a quarter degree: a camera turned more pulled the heading toward its own, and the
estimate followed it.

Position error, RMS, on the tuning set, before that change and after, with the cameras as the
simulator has them and with worse cameras (`--cameras=pessimistic`):

| Scenario | Ours, before | Ours | WPILib's | Worse cameras: ours, before | Ours | WPILib's |
|---|---|---|---|---|---|---|
| Clean driving, short route | 0.7 cm | 0.7 cm | 0.7 cm | 1.9 cm | 1.1 cm | 1.3 cm |
| Clean driving, full match | 0.7 cm | 0.7 cm | 0.8 cm | 2.0 cm | 1.3 cm | 1.5 cm |
| A camera misreading a tag, repeatedly | 1.3 cm | 1.3 cm | 1.2 cm | 2.3 cm | 1.5 cm | 1.4 cm |
| A camera frozen on an old image | 0.7 cm | 0.7 cm | 0.7 cm | 1.8 cm | 1.1 cm | 1.4 cm |
| Low grip | 1.6 cm | 1.6 cm | 4.1 cm | 2.5 cm | 1.9 cm | 4.5 cm |
| A stale wheel encoder | 2.5 cm | 2.5 cm | 3.7 cm | 3.0 cm | 2.7 cm | 4.0 cm |
| Gyro reading 3% high | 0.8 cm | 0.8 cm | 0.8 cm | 1.9 cm | 1.2 cm | 1.4 cm |
| A camera knocked on its mount | 2.6 cm | 1.4 cm | 1.4 cm | 3.2 cm | 1.6 cm | 1.8 cm |
| A camera's clock 20 ms off | 1.4 cm | 1.4 cm | 0.7 cm | 2.1 cm | 1.6 cm | 1.3 cm |
| Carried while disabled | 12.7 cm | 12.7 cm | 19.9 cm | 16.9 cm | 16.9 cm | 45.8 cm |
| A match with several faults in a row | 2.7 cm | 2.6 cm | 2.7 cm | 3.4 cm | 2.8 cm | 3.1 cm |

And on the held-out set, looked at once after the change in each condition:

| Scenario | Ours, before | Ours | WPILib's | Worse cameras: ours, before | Ours | WPILib's |
|---|---|---|---|---|---|---|
| Gyro reading 1% high | 0.7 cm | 0.7 cm | 0.7 cm | 2.0 cm | 1.1 cm | 1.3 cm |
| Gyro reading 6% high | 1.0 cm | 1.1 cm | 1.0 cm | 2.1 cm | 1.4 cm | 1.4 cm |
| Gyro reading low | 0.7 cm | 0.7 cm | 0.8 cm | 1.9 cm | 1.1 cm | 1.3 cm |
| Gyro drifting 30° a minute | 1.7 cm | 1.5 cm | 0.8 cm | 2.3 cm | 1.7 cm | 1.3 cm |
| A camera turned 1.5° on its mount | 1.5 cm | 0.9 cm | 0.9 cm | 2.6 cm | 1.3 cm | 1.4 cm |
| A camera turned 6° on its mount | 16.9 cm | 1.7 cm | 2.3 cm | 14.4 cm | 2.0 cm | 2.5 cm |
| A camera pitched 3° on its mount | 1.1 cm | 0.9 cm | 0.9 cm | 2.2 cm | 1.3 cm | 1.4 cm |
| A camera's clock 10 ms off | 1.1 cm | 1.1 cm | 0.7 cm | 2.2 cm | 1.5 cm | 1.3 cm |
| A camera's clock 40 ms off | 1.9 cm | 1.9 cm | 0.8 cm | 2.7 cm | 2.2 cm | 1.4 cm |
| Gyro reading high and a camera knocked | 2.6 cm | 1.5 cm | 1.4 cm | 3.2 cm | 1.7 cm | 1.8 cm |
| A clock off, on low grip | 1.4 cm | 1.4 cm | 5.1 cm | 2.7 cm | 1.9 cm | 5.3 cm |
| Every camera reading range long | 6.4 cm | 6.9 cm | 5.6 cm | 6.5 cm | 6.6 cm | 6.0 cm |
| Cameras whose noise is shared | 4.9 cm | 5.7 cm | 5.0 cm | 5.4 cm | 5.7 cm | 5.1 cm |
| Two cameras | 1.1 cm | 1.1 cm | 1.0 cm | 3.0 cm | 1.8 cm | 1.9 cm |
| A faster route | 0.8 cm | 0.9 cm | 1.4 cm | 2.3 cm | 1.3 cm | 1.9 cm |
| Far from the tags | 0.5 cm | 0.5 cm | 0.8 cm | 1.9 cm | 0.9 cm | 1.5 cm |
| Clean match, mounts off | 1.9 cm | 1.5 cm | 2.0 cm | 3.3 cm | 1.8 cm | 2.2 cm |
| Clean match, tags off | 1.7 cm | 1.9 cm | 19.5 cm | 2.9 cm | 2.2 cm | 20.7 cm |
| Clean match, 5% of images flipped | 0.8 cm | 0.8 cm | 1.1 cm | 2.5 cm | 1.1 cm | 1.6 cm |

WPILib's columns are this run's: the robot drives on our heading, so its path, and WPILib's
error along it, moves a little with ours. With worse cameras, ours was ahead of WPILib's in clean
driving on 16 of 20 seeds of the short route and 18 of 20 of the match, where before it was
behind by about half a centimeter. With the cameras as the simulator has them, no comparison that
was ours or the same became WPILib's on the tuning set; on the held-out set one did, every camera
reading range long (6.9 cm against 5.6, where it was 6.4 and counted the same): a range error is
not a turn, and the turns learned a little of it.

Where ours is ahead: wheels that slip (low grip, a stale encoder, a clock off on low grip),
being carried while disabled, a shove's and a collision's position error, tags sitting off the
layout, and a camera that keeps taking a tag's two poses the wrong way round (0.7 cm against 1.3,
and its largest jump 1.2 cm against 9.8). Since it learns the cameras' turns, a camera turned on
its mount, or knocked, no longer puts it behind: turned 6° (held out), 1.7 cm against 2.3 cm and
0.45° against 1.64°, where it was 16.9 cm and 2.0°; knocked, 1.4 cm against 1.4 cm, where it was
2.6 cm.

**A camera that keeps misreading a tag.** One camera reads its nearest tag as another of the
field's, 60 images running. Where it happens, beside the hub on the way to the trench, the
misreading camera is the only one with a tag in view. Before the change below, ours took the
estimate 2.65 m away within four loops of the first misread and held it there for 0.6 s (32 cm
RMS over the run against 1.2 cm): it rejected the first three misread images, and since they
agreed with each other, it took them for a shove and recovered to them. Now its position is 1.3
cm off against 1.2 cm, and its largest jump 2.3 cm against 10.0.

The change is to what a recovery needs. Rejected images that agree with each other move the
estimate only when they agree for independent reasons: two cameras, two tags, or a sign from the
robot's other sensors that it may have been moved where its wheels could not see (the IMU felt a
jolt or saw the wheels slide, the robot tilted past where its wheels' travel counts, it was
disabled, or it was placed and no camera has backed that yet). A tag misread as another puts
every image off by the same distance in the same direction, and that offset moves with the robot
exactly as a shove's would, so no test of one camera's images against the robot's motion tells
them apart; a second camera, a second tag, or the IMU does. A heading needs two cameras or such a
sign, since every tag a camera sees is turned by that camera's mount alike. In this study it
changed nothing but the misread, to the last digit: every other recovery already had a second
camera, a second tag or a sign behind it (the shove the study gives without a jolt is seen by a
camera reading two tags). The rule has no setting of its own.

Where WPILib's is ahead:

- **A camera's clock off.** 20 ms off, 1.4 cm against 0.7 cm, the same under the 1 cm rule and
  twice as far; 40 ms off (held out), 1.9 cm against 0.8 cm.
- **The gyro drifting 30° a minute** (held out). Ours is 2.1 cm off at rest near a tag against
  0.3 cm, and 1.5 cm RMS against 0.8 cm. With the gyro unplugged it is 1.5 cm off at rest against
  0.3 cm.
- **The largest jump.** Ours moves the estimate in one step when it decides the estimate is wrong:
  61 cm against 6 cm after a shove, 47 against 6 after a collision, 50 against 47 placed wrong,
  21 against 10 in the match with several faults, 3 m against 1.3 m carried while disabled. After
  the cameras were blind for 3 s and 10 s it steps 3 and 7 cm against 1 and 2 cm, and after the
  gyro restarts 4 cm against 1 cm. This is the design: a recovery is one move, and WPILib's
  estimator walks there over several images. An auto-align that follows the estimate feels the
  step. Where the robot is tilted it steps 44 cm against 15, and that is no recovery: riding up on
  a game piece its wheels slip, odometry runs ahead by up to half a meter while no camera sees a
  tag for half a second, its uncertainty grows to match, and the first image back is accepted and
  pulls the estimate the whole way (in the seed traced, from 50 cm off to 4). With the rear-half cameras some camera
  saw a tag throughout, and the step was 3 cm.
- **Every camera reading range long** (held out): 6.9 cm against 5.6 cm.

One fact about the layout stands behind several of these. Cameras at the corners, facing 90° apart
with a view 71° wide, never see the same tag at once: over a grid of poses across the field, no
pose has a tag in two cameras' images, where the earlier layout had one at half of them. So no
camera's image of a tag is ever held against another camera's image of the same tag, and more
often only one camera sees any tag at all. The misread and the tilt are each a camera alone with
its tags. A turned camera was too, until the gyro was made to carry the heading from one camera's
images to the next's to tell the cameras' turns apart.

Whether ours tells the truth about its uncertainty, as the mean share of loops past its 95% bound
over the tuning set: 0.3% for position and 0.3% for heading (0.6% and 1.2% before the turns were
learned); with worse cameras, 0.3% and 0.5% (1.0% and 6.3% before, when its heading was past its
bound in more than 5% of loops in 17 of the 41 scenarios, claiming about half the heading doubt a
camera turned 1° costs). With worse cameras its heading is now past the bound in more than 5% of
loops in one scenario, both IMUs unplugged (11%), and in most it is past it in none: what the
cameras' turns share is carried at the mounts' full tolerance, wider than the simulator's mounts
are off on most seeds. On the held-out set, with every camera reading range long, its position is
past its bound in 70% of loops, and with shared noise 38%, as before; with a camera turned 6° its
heading is past it in 19% (66% before), a turn six times its mount's tolerance; with the gyro
drifting, 8%.

Ours costs more to run: a median of 14 µs a loop (10 µs before it learned the cameras' turns) and
2.6 kB allocated a loop, against 2.9 µs for WPILib's filter given measurements already solved.

## What it can't make equal

- B's measurements are solved by A's pipeline with A's heading: where A's heading is off, the
  positions B is given are off with it, and WPILib's estimator can't use what A's does about that.
- The robot drives field-relative on A's heading, so the path every estimator is scored on depends a
  little on A.
- The yaw WPILib's estimators get already switches to the backup IMU and carries across the change,
  which is the study's doing, not WPILib's.

## What it doesn't show

It is a simulation. Its cameras are PhotonVision's simulated ones: every tag in view is found, with
a quarter pixel of corner noise, no motion blur, no lighting, and a perfect lens model. Its wheel
slip, shoves, and collisions are the simulator's models of them. A real robot's logs against an
independent truth are what would settle the question.

Four things flatter the changes that corrected the gyro and the cameras, and all four are the
simulator's:

- The simulated gyro's fault is a constant scale or a constant drift, which is exactly what the
  estimator now estimates. A real gyro's errors change with temperature and with how it is shaken.
- The simulated cameras scatter far less than they claim, so a camera a few centimeters off stands
  out within a second. A real camera's scatter is closer to its error.
- The watch on the cameras was developed against the simulator's knocked camera and its late
  clock. The held-out set varies their sizes, not their kind.
- A simulated camera's mount error is fixed for the run, or changes once when it is knocked, and
  the estimator learns a fixed turn. A real mount may flex as the robot accelerates or creep under
  vibration.
