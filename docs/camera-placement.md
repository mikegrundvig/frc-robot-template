# Camera placement

Where to mount the robot's AprilTag cameras, and how many, so the robot knows where it is anywhere
on the field. The CAD team needs that answer before the robot is designed, which means at kickoff.

The camera placement study answers it in simulation. It tries many camera layouts on the robot and
scores each one all over the field: how often its cameras see tags, and how close the robot's own
vision pipeline puts the robot from what they see. It knows nothing about any game, so it runs on
any field: last season's until the new one is published, then the new one the same day.

## What it answers

- **How many cameras**, and what each one more buys.
- **Where:** which corners or sides of the frame, how high, turned which way, and tilted how far up.
- **How the robot's own layout compares** with the others.

It doesn't know the robot's mechanisms. A mount it likes still needs a clear view past every
mechanism through its whole travel, and a rigid mount; that's for CAD to check.

## How it works

**Where the robot might be.** Positions on a grid over the whole field, every 0.5 m, kept half a
robot's length off the walls, each at 8 headings. Every spot counts the same, because the study
can't know what the next game rewards; a team that does can say so with weights (below). A spot the
robot can't reach, inside a field element, counts too until a weights file leaves it out.

**The candidates.** A candidate is a set of cameras, each placed and pointed as a `CameraConfig`
describes it: position, tilt up, and yaw. Every candidate uses the robot's own camera sensor. The
study tries:

- the robot's own cameras, as `Robots` configures them;
- a family built around the robot's frame: one to four cameras at its corners and the middles of
  its sides, facing out (ahead, back, sideways, or splayed 15°, 30°, or 45° from ahead or back), at
  8, 16, and 24 in off the floor, tilted up 0°, 10°, 20°, or 30°.

Every camera must be within the robot: inside the bumpers, and between the floor and the robot's
height (`Body.contains`), the check the robot's own configuration makes. A candidate with a camera
outside is refused, naming it. The family leaves out heights the robot doesn't reach.

**One set of images at each pose.** At each pose, every camera takes an image in PhotonVision's
simulation, with the camera model the simulator uses (`VisionSim`): the sensor's resolution and
field of view, about a quarter pixel of noise on each tag corner, and tags seen up to 7 m. Each
image goes through the robot's own vision pipeline (`VisionPipeline`), as on the robot: solved with
the gyro's heading, correcting heading from a multi-tag solution when it qualifies, with the same
filters and uncertainties. The robot stands still and level, and its gyro's heading is off by a
small random amount (a quarter degree, typically), as a real one is. The measurements from all the
cameras are combined as the estimator combines them, each weighed by its uncertainty. Each camera
takes two images at each pose, each with its own noise.

**Scores.** For each candidate, each a share of the field (of the poses, weighted):

| Score | The share of poses where | Why it matters |
|---|---|---|
| localized | the combined measurement is within 5 cm of where the robot really is | The ranking: seeing tags and placing the robot well, in one number |
| one tag | a camera sees at least one tag | Without a tag, the robot runs on its wheels alone |
| two tags | one camera sees two or more tags at once | A multi-tag solve needs two, and so does correcting the gyro's heading |
| two cameras | two or more cameras see tags | A blocked or knocked camera doesn't leave the robot blind |
| measured | the pipeline makes a measurement: tags in range, and the solve checks out | What reaches the estimator |
| heading | a measurement corrects the gyro's heading | What keeps the gyro honest over a match |

And the accuracy where there's a measurement: the position error (its root mean square, and the 95th
percentile) and the heading error.

Candidates rank by their localized share, then with fewer cameras first, then by position error.

**Deterministic.** Every random draw comes from the seed, so the same run gives the same report, to
the digit, on any computer with any number of threads.

## Running it

```bash
./gradlew cameraStudy                                   # the robot's cameras and the family, on its field
./gradlew cameraStudy --args="--weights=weights.csv"    # some spots count more
./gradlew cameraStudy --args="--layout=2027-field.json" # a tag layout, before its game is written
```

| Option | Default | What |
|---|---|---|
| `--robot=NAME` | `Robots.current()` | Which of `Robots.all()` to place cameras on |
| `--layout=FILE` | the robot's field (`Robot.field`) | A tag layout, as WPILib or FIRST publishes it (JSON) |
| `--weights=FILE` | none | Regions of the field that count more, or not at all |
| `--grid-meters=M` | 0.5 | Spacing of the positions |
| `--headings=N` | 8 | Headings at each position |
| `--frames=N` | 2 | Images each camera takes at each pose, each with its own noise |
| `--target-meters=M` | 0.05 | How close counts as localized |
| `--gyro-error-degrees=D` | 0.25 | The gyro heading's typical error |
| `--seed=N` | 1 | The seed every random draw comes from |
| `--threads=N` | 4 | How many threads to run on (the computer may be shared) |
| `--out=DIR` | `build/studies/camera-placement` | Where the report goes |

## Reading the result

The report is in `build/studies/camera-placement/`, written over on each run:

- `summary.md`: what ran, the top of the ranking, the best layout at each camera count, where the
  robot's own layout ranks, and a map of the field for the robot's layout and the best one.
- `candidates.csv`: every candidate, ranked, with every score, for a spreadsheet.
- `poses.csv`: every pose for the robot's layout and the best one, for a closer look.

A map shows, for each spot on the field, the share of headings where the robot is localized: `.`
for none, `1` to `9` for tenths, `#` for all. Blue's alliance wall is on the left.

To turn it into requirements for CAD: take the fewest cameras whose localized share is within a
few points of the best, then read off what the top layouts at that count share (corners or sides,
height, splay, tilt). Those are the requirements: "four cameras, one at each corner, facing out at
30° to 45°, 16 to 24 in up, tilted up 10° to 20°". Rerun with weights once the strategy says where
the robot must know where it is.

## Weights

A CSV file of regions, each written for the blue alliance, in meters in the field's frame (see
`frc.robot.lib.field`). The study adds each region's image on red's half by the field's symmetry.
A pose counts as the product of the weights of the regions it's in, and 1 in none: 0 leaves a spot
out (inside a field element), 3 counts it three times (where the robot scores).

```csv
# name, x min, y min, x max, y max, weight
Hub, 4.03, 3.43, 5.22, 4.64, 0
Scoring, 2.0, 2.5, 4.0, 5.5, 3
```

## At kickoff

1. Download the new field's tag layout (JSON), from WPILib or FIRST.
2. Run `./gradlew cameraStudy --args="--layout=<the file>"`. The field's symmetry is read from its
   tags.
3. Add weights once the strategy says where the robot must localize well, and run it again.
4. Once the new game's class is written (`docs/kickoff.md`), the study uses it without `--layout`.

## What it leaves out

- **Anything blocking a tag:** field elements, other robots, the robot's own mechanisms. Every
  layout's scores are an upper bound.
- **Motion:** the robot stands still, so there's no blur and no latency. Motion costs every layout,
  more so for far tags.
- **Tilt:** the robot is level.
- **Time:** the estimator combines many images over time, and the wheels between them, so the
  robot does better than one set of images. The study compares layouts; it doesn't predict the
  robot's accuracy on the field.
- **The lens:** a perfect lens, with PhotonVision's model of corner noise. Measure a real camera's
  noise before trusting the absolute numbers.
