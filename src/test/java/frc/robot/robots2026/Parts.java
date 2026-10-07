package frc.robot.robots2026;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Transmission;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * What the 2026 robots' fixtures are built from: the model's records with the defaults these
 * fixtures share, so each fixture reads as its robot's numbers. Nothing here is a fact about a
 * robot.
 */
final class Parts {
  private Parts() {}

  /** Meters in an inch: the teams' code measures in inches as often as in meters. */
  static final double IN = 0.0254;

  static Translation3d point(double x, double y, double z) {
    return new Translation3d(x, y, z);
  }

  static Translation3d inches(double x, double y, double z) {
    return new Translation3d(x * IN, y * IN, z * IN);
  }

  /** A joint's named outputs changed, the rest as it is: a team's value the model refuses. */
  static Joint.Revolute withOutputs(Joint.Revolute joint, Output... outputs) {
    return JointRevoluteBuilder.builder(joint).outputs(List.of(outputs)).build();
  }

  /**
   * A robot with one of its joints, and the drive on it, swapped for others, and none of its
   * configurations, groups or interlocks, which name the old joint: where a team's value the model
   * refuses is tried.
   */
  static RobotDefinition swapped(
      RobotDefinition robot, Joint old, Joint joint, Drive oldDrive, Drive drive) {
    return RobotDefinitionBuilder.builder(robot)
        .joints(robot.joints().stream().map(each -> each.equals(old) ? joint : each).toList())
        .drives(robot.drives().stream().map(each -> each.equals(oldDrive) ? drive : each).toList())
        .configurations(List.of())
        .interlocks(List.of())
        .build();
  }

  /** A corner's module, its wheel this many inches forward and left of the center. */
  static DrivetrainGeometry.Corner corner(
      DrivetrainGeometry.ModuleType module, double forward, double left) {
    return DrivetrainGeometry.Corner.builder()
        .module(module)
        .position(new Translation2d(forward * IN, left * IN))
        .build();
  }

  /** A team's yaw inertia alone, in kg m²: the model's box gives the pitch and roll. */
  static Body.Turning yaw(double kilogramSquareMeters) {
    return Body.Turning.yaw(KilogramSquareMeters.of(kilogramSquareMeters));
  }

  /** A gearing with its losses not yet measured. */
  static Gearing gearing(double ratio) {
    return Gearing.builder().ratio(ratio).efficiency(1.0).build();
  }

  static Drive drive(String label, Actuator actuator, Transmission transmission) {
    return Drive.builder().label(label).actuator(actuator).transmission(transmission).build();
  }

  static Transmission.Term term(Joint joint, double ratio) {
    return Transmission.Term.builder()
        .joint(joint)
        .ratio(ratio)
        .pitchRadius(Optional.empty())
        .build();
  }

  /** A prismatic joint's term: its turns for each of its drum's, and the drum's pitch radius. */
  static Transmission.Term term(Joint.Prismatic joint, double ratio, Distance pitchRadius) {
    return Transmission.Term.builder()
        .joint(joint)
        .ratio(ratio)
        .pitchRadius(Optional.of(pitchRadius))
        .build();
  }

  /** A CANcoder's resolution: 4096 counts a turn. */
  static final int CANCODER = 4096;

  /** An encoder on one joint, at a resolution. */
  static Sensor.Encoder encoder(
      String label, Joint joint, double ratio, boolean absolute, int countsPerTurn) {
    return Sensor.Encoder.builder()
        .label(label)
        .terms(List.of(term(joint, ratio)))
        .absolute(absolute)
        .countsPerTurn(Optional.of(countsPerTurn))
        .build();
  }

  /**
   * A body, in inches: its frame, bumpers, weight, center of mass and inertia, its yaw alone where
   * a team's code holds one.
   */
  static Body body(
      double frameLength,
      double frameWidth,
      double height,
      double bumperEnds,
      double bumperSides,
      double bumperBottom,
      double bumperTop,
      Body.Weight weight,
      Translation3d cg,
      Body.Turning inertia) {
    return Body.builder()
        .frameLength(Inches.of(frameLength))
        .frameWidth(Inches.of(frameWidth))
        .height(Inches.of(height))
        .bumperEnds(Inches.of(bumperEnds))
        .bumperSides(Inches.of(bumperSides))
        .bumperBottom(Inches.of(bumperBottom))
        .bumperTop(Inches.of(bumperTop))
        .weight(weight)
        .cg(Optional.of(cg))
        .inertia(inertia)
        .build();
  }

  /**
   * A camera's sensor, from what the team's code gives (its image size, and its field of view,
   * horizontal or diagonal as the team's code takes it), with the template's guesses for the rest:
   * 30 fps, 30 ms, a 7 m sight.
   */
  static CameraConfig.Sensor sensor(
      int width, int height, double fieldOfViewDegrees, CameraConfig.Across across) {
    return CameraConfig.Sensor.builder()
        .widthPixels(width)
        .heightPixels(height)
        .fieldOfView(Degrees.of(fieldOfViewDegrees))
        .fieldOfViewAcross(across)
        .frameRate(Hertz.of(30))
        .latency(Milliseconds.of(30))
        .sightRange(Meters.of(7))
        .build();
  }

  /** A camera, in inches and degrees, as the model names them: left, and tilted up. */
  static CameraConfig camera(
      String name,
      Translation3d at,
      double roll,
      double tiltUp,
      double yaw,
      CameraConfig.Sensor sensor,
      Optional<Joint> joint) {
    return CameraConfig.builder()
        .name(name)
        .forward(Meters.of(at.getX()))
        .left(Meters.of(at.getY()))
        .up(Meters.of(at.getZ()))
        .roll(Degrees.of(roll))
        .tiltUp(Degrees.of(tiltUp))
        .yaw(Degrees.of(yaw))
        .sensor(sensor)
        .noiseScale(1.0)
        .joint(joint)
        .build();
  }
}
