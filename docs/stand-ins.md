# Stand-ins

Parts of this template exist only because the real 2027 version hasn't shipped yet. They are
throwaway: when the real one arrives, delete ours and use it. Add a row when you add a stand-in, and
delete the row when you replace it.

| Stand-in | Stands in for | Replace when |
|---|---|---|
| The swerve drivetrain, `src/main/java/frc/robot/subsystems/drive/` | AdvantageKit's swerve templates, which also bring the motor-controller and gyro code for real hardware, and the thread that reads them at 250 Hz. Our IO already carries 250 Hz readings the way theirs does, and the simulator produces them, so their hardware code can feed the estimator unchanged. | AdvantageKit publishes its 2027 swerve templates, planned for WPILib's 2027 beta (AdvantageKit 27.0.0-alpha-6 release notes) |

## Waiting on

Not built here at all, because the library has no 2027 build yet. Add these when it does.

- **Autos:** PathPlannerLib. (ChoreoLib already has a 2027 alpha build.)
- **Vision:** PhotonLib. Its one 2027 release (alpha-2, May 2026) targets WPILib alpha-5, and
  GradleRIO alpha-7 refuses it. PhotonVision's development builds of its main branch target
  alpha-7 and publish their own vendordep (`maven.photonvision.org/snapshots`, `photonlib-json`):
  in a simulation spike (2026-09-29) a simulated camera saw the tags, and the multi-tag, single-tag,
  and heading-constrained solves found the robot to within a centimeter. Vision can pin one of
  those builds until a release; the coprocessor must run the same version.
- **Physics simulation** (wheel slip, collisions, game pieces): maple-sim. Until then, simulation
  tests the code, not the robot's traction or power. The simulator already keeps the robot's true
  pose apart from its sensors (`DriveSimulation`, `SensorErrors`), and a shove stands in for a
  collision; wheels still never slip.
