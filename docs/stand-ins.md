# Stand-ins

Parts of this template exist only because the real 2027 version hasn't shipped yet. They are
throwaway: when the real one arrives, delete ours and use it. Add a row when you add a stand-in, and
delete the row when you replace it.

| Stand-in | Stands in for | Replace when |
|---|---|---|
| The swerve drivetrain, `src/main/java/frc/robot/subsystems/drive/` | AdvantageKit's swerve templates, which also bring the motor-controller and gyro code for real hardware and high-rate odometry | AdvantageKit publishes its 2027 swerve templates, planned for WPILib's 2027 beta (AdvantageKit 27.0.0-alpha-6 release notes) |

## Waiting on

Not built here at all, because the library has no 2027 build yet. Add these when it does.

- **Autos:** PathPlannerLib. (ChoreoLib already has a 2027 alpha build.)
- **Physics simulation** (wheel slip, collisions, game pieces): maple-sim. Until then, simulation
  tests the code, not the robot's traction or power.
