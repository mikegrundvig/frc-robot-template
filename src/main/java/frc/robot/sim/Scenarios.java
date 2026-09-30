package frc.robot.sim;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.Bump;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The simulator's scenarios, by name: {@code ./gradlew run -Pscenario=<name>} starts the simulator
 * in one, and {@code ./gradlew scenarios} lists them. Each is a fault or a condition the simulator
 * can already produce, written for any game: a camera names the robot's first, and a bump is the
 * field's nearest.
 *
 * <p>To add one, add it to {@link #ALL} with what it does to the robot. What it does must be
 * something the simulated world can do ({@link SensorErrors}, {@link DriveSimulation}); a fault it
 * can't make yet starts there; docs/stand-ins.md lists those.
 */
public final class Scenarios {
  /** A camera knocked 4 in down on its mount. */
  static final Transform3d KNOCKED_LOW =
      new Transform3d(Inches.of(0), Inches.of(0), Inches.of(-4), Rotation3d.ZERO);

  /** A camera turned 2° on its mount: small enough to miss by eye. */
  static final Transform3d TURNED =
      new Transform3d(
          Inches.of(0),
          Inches.of(0),
          Inches.of(0),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(2)));

  /** How far a collision or a push moves the robot: to its left. */
  static final Transform2d KNOCKED_SIDEWAYS = new Transform2d(0, 0.6, Rotation2d.ZERO);

  /** How far short of a bump's ramp the robot is set down, from its center: clear of the ramp. */
  static final double BUMP_CLEARANCE_METERS = 1.0;

  /** Every scenario: faults in the robot's sensors first, then what can happen to it. */
  public static final List<Scenario> ALL =
      List.of(
          Scenario.builder()
              .name("camera-low")
              .description("The robot's first camera knocked 4 in low on its mount.")
              .sensorFaults(
                  (errors, cameras) -> errors.withCameraMountError(first(cameras), KNOCKED_LOW))
              .event(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("camera-turned")
              .description("The robot's first camera turned 2° on its mount.")
              .sensorFaults(
                  (errors, cameras) -> errors.withCameraMountError(first(cameras), TURNED))
              .event(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("gyro-scale")
              .description(
                  "The gyro reads 15% more turn than the robot makes; the backup IMU doesn't.")
              .sensorFaults(
                  (errors, cameras) ->
                      errors.withGyro(
                          SensorErrorsGyroErrorsBuilder.builder(errors.gyro()).scale(1.15).build()))
              .event(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("worn-tread")
              .description(
                  "Every wheel's tread worn 3%: the robot travels 3% less than odometry says.")
              .sensorFaults((errors, cameras) -> errors.withWheelRadiusScale(0.97))
              .event(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("module-zero-off")
              .description(
                  "The front left module's absolute encoder zeroed 5° off: it points 5° from where"
                      + " the code thinks.")
              .sensorFaults((errors, cameras) -> errors.withSteerOffset(0, 5.0))
              .event(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("exact-sensors")
              .description(
                  "Wheels and sensors exactly as configured, not a season-worn robot's: odometry is"
                      + " the truth.")
              .sensorFaults((errors, cameras) -> SensorErrors.NONE)
              .event(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("collision")
              .description(
                  "Another robot hits it: knocked 0.6 m to its left, a jolt its IMU feels.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(world -> world.drivetrain().hit(KNOCKED_SIDEWAYS))
              .build(),
          Scenario.builder()
              .name("shove")
              .description(
                  "Pushed 0.6 m to its left without a jolt: neither its wheels nor its IMU notice.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(world -> world.drivetrain().shove(KNOCKED_SIDEWAYS))
              .build(),
          Scenario.builder()
              .name("wheel-slip")
              .description(
                  "The front left wheel spins half a turn without moving the robot: its encoder"
                      + " counts travel the floor never saw.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(world -> world.drivetrain().spin(0, Math.PI))
              .build(),
          // An approximation: the simulator can tilt the robot and scale how far all its wheels
          // move
          // it, but can't lift one wheel. A lifted wheel would spin free and disagree with the
          // others; these all slip alike, and the robot isn't raised.
          Scenario.builder()
              .name("wheel-lifted")
              .description(
                  "Its front left corner up on a game piece: tilted 5° nose up and 5° left side up,"
                      + " moving three quarters as far as its wheels turn, for the rest of the run"
                      + " (the tilt holds over any bump). Approximate: every wheel slips alike, so"
                      + " they never disagree as a lifted one would; the robot isn't raised.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(
                  world -> {
                    // WPILib's convention: nose up is a negative pitch, left side up a positive
                    // roll.
                    // A tilt by hand holds until the terrain is set again, bumps or not.
                    world.drivetrain().tilt(Math.toRadians(-5), Math.toRadians(5));
                    world.drivetrain().grip(0.75);
                  })
              .build(),
          Scenario.builder()
              .name("gyro-unplugged")
              .description("The gyro unplugged: the backup IMU has to keep the heading.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(world -> world.drivetrain().disconnectGyro(true))
              .build(),
          Scenario.builder()
              .name("gyro-frozen")
              .description("The gyro freezes: it keeps reporting, the same yaw every time.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(world -> world.drivetrain().freezeGyro(true))
              .build(),
          Scenario.builder()
              .name("imus-unplugged")
              .description(
                  "The gyro and the backup IMU both unplugged: the heading rests on the wheels"
                      + " alone.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(
                  world -> {
                    world.drivetrain().disconnectGyro(true);
                    world.drivetrain().disconnectBackupGyro(true);
                  })
              .build(),
          Scenario.builder()
              .name("bump-crossing")
              .description(
                  "Carried to 1 m short of the nearest bump's ramp, facing across it, to drive"
                      + " over. Its gyro feels the turn but its wheels see no move, so only its"
                      + " cameras can say where it is. Needs a field with bumps.")
              .sensorFaults(Scenario.NO_FAULTS)
              .event(Scenarios::beforeTheNearestBump)
              .build());

  private Scenarios() {}

  /**
   * The scenario with this name.
   *
   * @throws IllegalArgumentException naming every scenario there is, when none has this name
   */
  public static Scenario named(String name) {
    for (Scenario scenario : ALL) {
      if (scenario.name().equals(name)) {
        return scenario;
      }
    }
    throw new IllegalArgumentException(
        "No simulator scenario is named \""
            + name
            + "\". The scenarios: "
            + ALL.stream().map(Scenario::name).collect(Collectors.joining(", ")));
  }

  /** Every scenario, one a line: its name, then what it does. */
  static String listing() {
    return ALL.stream().map(Scenarios::line).collect(Collectors.joining());
  }

  /** A scenario's line in the list. */
  static String line(Scenario scenario) {
    return String.format(Locale.ROOT, "%-16s %s%n", scenario.name(), scenario.description());
  }

  /**
   * For the build: with no arguments, lists the scenarios ({@code ./gradlew scenarios}); with a
   * name, gives that scenario's line, or stops with the list when there is none by that name, so
   * {@code ./gradlew run -Pscenario=<name>} stops at a misspelled name before the simulator starts.
   */
  public static void main(String... args) {
    try {
      System.out.print(args.length == 0 ? listing() : line(named(args[0])));
    } catch (IllegalArgumentException e) {
      System.err.println(e.getMessage());
      System.exit(1);
    }
  }

  /** The robot's first camera, which the camera scenarios knock. */
  private static String first(List<CameraConfig> cameras) {
    if (cameras.isEmpty()) {
      throw new IllegalArgumentException("this scenario knocks a camera, and the robot has none");
    }
    return cameras.get(0).name();
  }

  /**
   * Puts the robot down short of the bump nearest it, on the side it's on, facing across the crest:
   * the drive team carrying it there. Its wheels don't turn, so odometry doesn't see the move; its
   * gyro feels any turn.
   *
   * @throws IllegalStateException on a field without bumps
   */
  static void beforeTheNearestBump(SimulatedWorld world) {
    Translation2d robot = world.truePose().getTranslation();
    Bump bump =
        world.drivetrain().terrain().bumps().stream()
            .min(Comparator.comparingDouble(each -> middle(each).getDistance(robot)))
            .orElseThrow(
                () ->
                    new IllegalStateException(
                        "bump-crossing needs a field with bumps, and this one has none"));
    boolean onTheAcrossSide = bump.acrossOffset(robot) >= 0;
    double out = bump.depthMeters() / 2 + BUMP_CLEARANCE_METERS;
    Rotation2d across = bump.across();
    Translation2d spot = middle(bump).plus(new Translation2d(onTheAcrossSide ? out : -out, across));
    world.place(new Pose2d(spot, onTheAcrossSide ? across.plus(Rotation2d.PI) : across));
  }

  /** The middle of a bump's crest. */
  private static Translation2d middle(Bump bump) {
    return bump.crestStart().interpolate(bump.crestEnd(), 0.5);
  }
}
