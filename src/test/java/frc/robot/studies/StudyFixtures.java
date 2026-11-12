package frc.robot.studies;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.List;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;

/**
 * A field that's easy to check by hand, and cameras to put on the robot. The field is 16 x 8 m,
 * with a wall of three tags on the blue alliance's wall, facing downfield, half a meter up and half
 * a meter apart, and nothing else.
 */
final class StudyFixtures {
  static final FieldModel WALL =
      new FieldModel(
          Rebuilt.GAME.layout(
              new Field(
                  "A wall of tags",
                  "2027",
                  "Test",
                  null,
                  16,
                  8,
                  "frc",
                  List.of(tag(1, 3.5), tag(2, 4.0), tag(3, 4.5)))),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());

  /** Three meters from the wall, facing it. */
  static final Sample FACING_THE_WALL =
      new Sample(new Pose2d(3, 4, Rotation2d.fromDegrees(180)), 1, 0, 0);

  /** The same spot, facing away. */
  static final Sample FACING_AWAY = new Sample(new Pose2d(3, 4, Rotation2d.ZERO), 1, 1, 0);

  static final CameraConfig.ImageSensor SENSOR =
      ExampleRobot.SIM.definition().cameras().get(0).sensor();

  private StudyFixtures() {}

  private static FieldTag tag(int id, double y) {
    return new FieldTag(id, new Pose3d(0, y, 0.5, Rotation3d.ZERO));
  }

  /**
   * A camera 0.4 m up and level, at the robot's middle side to side, facing the way it's turned.
   */
  static CameraConfig camera(String name, double forwardMeters, double yawDegrees) {
    return CameraConfig.of(name, SENSOR)
        .at(Meters.of(forwardMeters), Meters.of(0), Meters.of(0.4))
        .facing(Degrees.of(0), Degrees.of(yawDegrees))
        .roll(Degrees.of(0))
        .noiseScale(1.0)
        .measured();
  }

  static final CameraConfig FRONT = camera("front", 0.3, 0);
  static final CameraConfig BACK = camera("back", -0.3, 180);

  static final CameraStudyConfig ONE_THREAD =
      CameraStudyConfigBuilder.builder(CameraStudyConfig.defaults()).threads(1).build();
}
