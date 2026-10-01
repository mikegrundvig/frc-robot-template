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
robot's length off the walls, each at 8 headings. Each position's headings start at their own
random fraction of the 45° step, so they don't all line up with the walls and with cameras splayed
45°, which would favor the layouts that happen to match them. Every spot counts the same, because the study
can't know what the next game rewards; a team that does can say so with weights (below). A spot the
robot can't reach, inside a field element, counts too until a weights file leaves it out.

**The candidates.** A candidate is a set of cameras, each placed and pointed as a `CameraConfig`
describes it: position, tilt up, and yaw. Every candidate uses the robot's own camera sensor. The
study tries:

- the robot's own cameras, as `Robots` configures them, always;
- every layout CAD's places allow, when CAD has said where cameras could go (`--mounts`, below);
- otherwise, a family built around the robot's frame: one to four cameras at its corners and the middles of
  its sides, facing out (ahead, back, sideways, or splayed 15°, 30°, or 45° from ahead or back), at
  8, 16, and 24 in off the floor, tilted up 0°, 10°, 20°, or 30°.

Every camera must be within the robot: inside the bumpers, and between the floor and the robot's
height (`Body.contains`), the check the robot's own configuration makes. A candidate with a camera
outside is refused, naming it. The family leaves out heights the robot doesn't reach.

**One set of images at each pose.** At each pose, every camera takes an image as PhotonVision's
simulation takes it, with the camera model the simulator uses (`VisionSim`): the sensor's
resolution and field of view, about a quarter pixel of noise on each tag corner, and tags seen up
to 7 m. Each image goes through the robot's own vision pipeline (`VisionPipeline`), as on the
robot: solved with the gyro's heading, correcting heading from a multi-tag solution when it
qualifies, with the same filters and uncertainties. The robot stands still and level, and its gyro's heading is off by a
small random amount (a quarter degree, typically), as a real one is. The measurements from all the
cameras are combined as the estimator combines them, each weighed by its uncertainty. Each camera
takes two images at each pose, each with its own noise.

**PhotonVision's simulation, without the pictures.** PhotonVision's simulation draws every image
into native memory that only the garbage collector frees: fine at the simulator's 30 images a
second, but the default run takes about a million and a half, and that exhausts a computer's
memory. So the study takes its images step for step as PhotonVision's does (`CameraImageSim`): the
same test of which tags are in view, the same projection, the same model of pixel noise, the same
detection limits, and PhotonVision's own solves, without drawing anything. Its test
(`CameraImageSimTest`) gives the two the same seeds, so they draw the same noise, and compares
them image for image over a sample of 2026's field: they must see the same tags and make the same
multi-tag solves, with corners within 0.01 px and solves within a millimeter and a milliradian. In
the last run, 160 images from the template robot's four cameras saw the same 274 tags and made the
same 75 multi-tag solves, with corners within 0.001 px and solves within 0.03 mm.

**The ranking: the error 95% of the field is within.** For each candidate, the study finds the
position error that 95% of the field (of the poses, weighted, and every set of images at them) is
within; a spot with no measurement at all counts as missed by any distance. Smaller is better. It
puts coverage and accuracy in one number that keeps telling layouts apart: a share within a fixed
distance stops doing that once every good layout reaches nearly all of a field rich in tags, as
2026's is. A layout that leaves more than 5% of the field without a measurement has no such number
("none"); those rank after the rest, by how much of the field they place within 5 cm. Errors within
a millimeter of each other count as a tie, about the sampling's own noise, and a tie goes to fewer
cameras.

**The other scores**, each a share of the field:

| Score | The share of poses where | Why it matters |
|---|---|---|
| within 2 cm, within 5 cm | the combined measurement is that close to where the robot really is | How much of the field is placed well, at two standards |
| one tag | a camera sees at least one tag | Without a tag, the robot runs on its wheels alone |
| two tags | one camera sees two or more tags at once | A multi-tag solve needs two, and so does correcting the gyro's heading |
| two cameras | two or more cameras see tags | A blocked or knocked camera doesn't leave the robot blind |
| measured | the pipeline makes a measurement: tags in range, and the solve checks out | What reaches the estimator |
| heading | a measurement corrects the gyro's heading | What keeps the gyro honest over a match |

And the accuracy where there's a measurement: the position error's root mean square, and the
heading error's.

**Deterministic.** Every random draw comes from the seed, so the same run gives the same report, to
the digit, on any computer with any number of threads. A camera's noise depends only on the seed,
the pose, and where the camera is, so a layout scores the same whatever else is tried beside it.

## Running it

```bash
./gradlew cameraStudy                                   # the robot's cameras and the family
./gradlew cameraStudy --args="--weights=weights.csv"    # some spots count more
./gradlew cameraStudy --args="--layout=2027-field.json" # a field WPILib doesn't have yet
```

The default run takes under a minute on two threads (45 to 50 s on the computer it was written
on, for 3,840 poses and 157 layouts), and under 1 GB of memory. A finer grid costs the square:
`--grid-meters=0.25` takes four times as long.

| Option | Default | What |
|---|---|---|
| `--robot=NAME` | `Robots.current()` | Which of `Robots.all()` to place cameras on |
| `--field=NAME` | the season's, `Fields.DEFAULT_FIELD` | One of WPILib's fields, by its name in `Fields`: `FRC_2026_REBUILT_ANDY_MARK` |
| `--layout=FILE` | none | A tag layout file (JSON), as WPILib or FIRST publishes it, for a field WPILib doesn't have yet |
| `--weights=FILE` | none | Regions of the field that count more, or not at all |
| `--mounts=FILE` | none: the family around the frame | Where CAD could put cameras, and the ways each could face (below) |
| `--cameras=N` | 4 | The most cameras in a layout |
| `--grid-meters=M` | 0.5 | Spacing of the positions |
| `--headings=N` | 8 | Headings at each position |
| `--frames=N` | 2 | Images each camera takes at each pose, each with its own noise |
| `--gyro-error-degrees=D` | 0.25 | The gyro heading's typical error |
| `--seed=N` | 1 | The seed every random draw comes from |
| `--threads=N` | 2 | How many threads to run on (the computer may be shared) |
| `--out=DIR` | `build/studies/camera-placement` | Where the report goes |

## Reading the result

The report is in `build/studies/camera-placement/`, written over on each run:

- `summary.md`: what ran, the top of the ranking, the best layout at each camera count, where the
  robot's own layout ranks, and a map of the field for the robot's layout and the best one.
- `candidates.csv`: every candidate, ranked, with every score, for a spreadsheet.
- `poses.csv`: every pose for the robot's layout and the best one, for a closer look.

A map shows, for each spot on the field, the share of headings where the robot is placed within
5 cm: `.` for none, `1` to `9` for tenths, `#` for all. Blue's alliance wall is on the left.

To turn it into requirements for CAD:

1. Take the fewest cameras whose 95% error is within a few millimeters of the best's. Errors a
   millimeter or two apart, or shares a point apart, are the sampling's own noise: choose among
   them on something else, such as easier mounts.
2. Among those, prefer a high "two cameras" share: one camera blocked by a mechanism, a robot, or
   a knock still leaves the robot seeing tags.
3. Read off what the top layouts at that count share (corners or sides, height, splay, tilt).
   Those are the requirements, in words like these: "four cameras, one at each corner, facing out
   at 30° to 45°, 16 to 24 in up, tilted up 10° to 20°".

Rerun with weights once the strategy says where the robot must know where it is.

## Designing with CAD

Once the robot is taking shape, the family's guesses give way to CAD's own places: where on this
robot a camera could actually go, clear of the mechanisms, on something rigid. CAD lists them in a
CSV file, one place a line, with the ways a camera there could face, and the study scores every
layout of one camera up to `--cameras` (four, unless you say otherwise) at different places, each
facing any of its place's ways:

```csv
# name,      forward, left,   up, yaws (left of ahead), tilts (up)
front-left,  12.25,   12.25,  20, 0 30 45,              10 20
front-right, 12.25,   -12.25, 20, 0 -30 -45,            10 20
back-left,   -12.25,  12.25,  20, 180 150 135,          10 20
back-right,  -12.25,  -12.25, 20, 180 -150 -135,        10 20
shooter-top, 0,       0,      28, 0 180,                15
```

Positions are in inches from the robot's origin (the frame's center, on the floor; forward, left,
and up, as `CameraConfig` measures them), and angles in degrees: a yaw of 0 faces ahead and
positive turns left; a tilt is how far up from level. Lists of yaws and tilts are separated by
spaces. Each place must be within the robot, or the study names it and stops.

Every way of facing multiplies the layouts: five places with six ways each make over 9,000 layouts
of up to four cameras, and the study scores at most 5,000 in one run, saying how many there were
when there are more. Offer the ways that are really in question, and narrow them run by run.

How CAD uses it while designing:

1. Early: list the places the design leaves room for, with the ways each could face. Run the study
   and read the top layouts at each camera count.
2. Settle the count, the places, and roughly the angles; narrow the file to those, with finer
   steps of angle, and run again.
3. When the design changes (a mechanism moves into a camera's view, a place goes away), change the
   file and run again. Each run takes a minute or two.
4. Commit the chosen layout as the robot's cameras in `Robots`: from then on it's "the robot's own"
   in every run, the baseline the alternatives are measured against.

## Weights

A CSV file of regions, each written for the blue alliance, in meters in the field's frame (see
`frc.robot.lib.field`). The study adds each region's image on red's half by the field's symmetry.
A pose counts as the product of the weights of the regions it's in, and 1 in none: 0 leaves a spot
out (inside a field element), 3 counts it three times (where the robot scores).

```csv
# name, x min, y min, x max, y max, weight
Field element, 7.5, 3.5, 9.0, 4.5, 0
Scoring, 1.0, 2.0, 3.0, 6.0, 3
```

## An example: the template's robot on 2026's field

2026's field stands in until 2027's is published, so this says nothing about next season's robot;
it shows what a run says and how to read it. The default run, seed 1:

| Cameras | The best layout of that many | Rank | Localized | Two tags | Two cameras |
|---:|---|---:|---:|---:|---:|
| 1 | ahead, 16 in up, tilted 20° | 138 | 63.3% | 45.8% | 0.0% |
| 2 | ahead and back, 24 in up, tilted 10° | 72 | 89.1% | 75.9% | 48.0% |
| 3 | front corners splayed 45°, and back, 16 in up, tilted 20° | 35 | 97.6% | 91.5% | 82.7% |
| 4 | corners splayed 30°, 24 in up, tilted 20° | 1 | 99.7% | 95.5% | 93.5% |
| 4 | the template robot's own | 26 | 98.6% | 93.2% | 96.2% |

Every camera up to four buys coverage; the fourth still buys two points localized and eleven of two
cameras seeing tags. The top 25 layouts are four cameras within about a point of each other:
corners splayed 30° or 45° and the four sides do alike, at any of the three heights, tilted up 10°
to 20° (level does a little worse). The template robot's own layout is among them. With seed 2,
95% of layouts move less than 0.3 points, and the top two stay the top two. REBUILT has 32 tags,
many in mid-field on the hubs; a field with fewer tags spreads the layouts further apart.

## At kickoff

The study needs only the field's size and tags, never the game's class (`docs/kickoff.md`), so it
runs the day the field is published.

1. Download the new field's tag layout (JSON), from WPILib or FIRST.
2. Run `./gradlew cameraStudy --args="--layout=<the file>"`. The field's symmetry is read from its
   tags.
3. Add weights once the strategy says where the robot must localize well, and run it again.
4. Once WPILib is updated for the season, its default field is the new one: no `--layout` needed.

To try a layout of your own (CAD's first draft, say), give it a robot of its own in `Robots.all()`
with those cameras, and run with `--robot=<its name>`: its cameras are "the robot's own".

## What it leaves out

- **Anything blocking a tag:** field elements, other robots, the robot's own mechanisms. Every
  layout's scores are an upper bound.
- **Motion:** the robot stands still, so there's no blur and no latency. Motion costs every layout,
  more so for far tags.
- **Tilt:** the robot is level.
- **Time:** the estimator combines many images over time, and the wheels between them, so the
  robot does better than one set of images. The study compares layouts; it doesn't predict the
  robot's accuracy on the field.
- **Small and failed tags:** a tag smaller than 100 square pixels in the image isn't detected,
  PhotonVision's default; a real camera's limit depends on its lens, exposure, and settings. A tag
  whose own solve fails is left out, as the robot leaves out a pose that isn't a number
  (PhotonVision's simulation would report it at the camera itself); none failed in the example run.
- **The lens:** a perfect lens, with PhotonVision's model of corner noise. Measure a real camera's
  noise before trusting the absolute numbers.
