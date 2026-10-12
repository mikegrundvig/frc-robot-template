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
```

Every scenario at 20 seeds is 820 simulated robots, more than one program's memory holds under a
2 GB heap: run it a few scenarios at a time, each to its own `--out`, and put the results together
with `--args="--merge=DIR,DIR,... --out=DIR"`.

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
interface ([runtime/docs/localizers.md](../runtime/docs/localizers.md)): A is the robot's own, run
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

## The measures

For each estimator in each run, against the true pose, over every loop from the first:

- the position error's RMS, 95th percentile, and worst, and the heading error's;
- the same over the 2 s after each fault;
- how long after a fault the estimate is back within 5 cm and 1° and stays there 0.2 s ("never" if
  not within the run);
- the largest jump: how far the estimate moved in one loop beyond how far the robot did (what a
  driver, or an auto-align, feels);
- the error at rest, over the last half second of each stop near a tag and far from tags.

The report gives each as its median across the seeds with the interquartile range. A verdict of
"same" means two medians differ by less than the wider interquartile range. The seeds change only
the sensors' noise, so that range is narrow: read the size of a difference, not only its verdict.

`--cost` plays one run's inputs again into a fresh estimator of each kind, alone on the thread, and
gives each loop's time and the bytes it allocated. A is timed from its drivetrain readings to its
update, given its pipeline's measurements; B the same; C and D with solving their images.

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
