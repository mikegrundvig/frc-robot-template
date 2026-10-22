package frc.robot.studies;

import static frc.robot.studies.Candidates.HEIGHTS;
import static frc.robot.studies.Candidates.SPLAYS;
import static frc.robot.studies.Candidates.TILTS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.groups.Tuple.tuple;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.world.robot.BodyBuilder;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

/** The layouts the study tries. */
class CandidatesTest {
  /**
   * The family's arrangements: one camera ahead; two at the front corners, two ahead and back, two
   * left and right; three; four at the corners; four on the sides. Those at corners, one per splay.
   */
  private static final int ARRANGEMENTS = 1 + SPLAYS.size() + 2 + SPLAYS.size() + SPLAYS.size() + 1;

  @Test
  void theRobotsOwnComesFirstThenTheFamily() {
    List<Candidate> candidates = Candidates.of(ExampleRobot.SIM);

    assertThat(candidates.get(0).cameras()).isEqualTo(ExampleRobot.SIM.definition().cameras());
    assertThat(candidates).hasSize(1 + ARRANGEMENTS * HEIGHTS.size() * TILTS.size());
    Set<String> names = new HashSet<>();
    for (Candidate candidate : candidates) {
      assertThat(names.add(candidate.name())).as(candidate.name()).isTrue();
      candidate.checkOn(ExampleRobot.SIM); // within the robot, or it throws
      assertThat(candidate.cameras())
          .extracting(CameraConfig::sensor)
          .containsOnly(ExampleRobot.SIM.definition().cameras().get(0).sensor());
    }
  }

  @Test
  void theFamilyFacesOutFromTheFrame() {
    Candidate corners =
        Candidates.of(ExampleRobot.SIM).stream()
            .filter(
                candidate ->
                    candidate
                        .name()
                        .equals("4 cameras: corners, splayed 30°, 16 in up, tilted 10°"))
            .findFirst()
            .orElseThrow();

    // Frame 27.5 in square: each corner 13.75 in out, less 1.5 in for the camera's case.
    assertThat(corners.cameras())
        .extracting(
            camera -> camera.name(),
            camera -> rounded(camera.forward().in(Inches)),
            camera -> rounded(camera.left().in(Inches)),
            camera -> rounded(camera.yaw().in(Degrees)))
        .containsExactly(
            tuple("front-left", 12.25, 12.25, 30.0),
            tuple("front-right", 12.25, -12.25, -30.0),
            tuple("back-left", -12.25, 12.25, 150.0),
            tuple("back-right", -12.25, -12.25, -150.0));
  }

  private static double rounded(double value) {
    return Math.round(value * 1000) / 1000.0;
  }

  @Test
  void heightsTheRobotDoesntReachAreLeftOut() {
    RobotConfig shortRobot =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .body(
                        BodyBuilder.builder(ExampleRobot.SIM.definition().body())
                            .overallHeight(Inches.of(20))
                            .build())
                    .cameras(List.of(StudyFixtures.FRONT))
                    .build())
            .build();

    // Only the two lower heights fit under 20 in.
    assertThat(Candidates.of(shortRobot)).hasSize(1 + ARRANGEMENTS * 2 * TILTS.size());
  }

  @Test
  void theFamilyLeavesOutLayoutsOfMoreCameras() {
    List<Candidate> upToTwo =
        Candidates.family(ExampleRobot.SIM.definition().body(), StudyFixtures.SENSOR, 2);

    // Ahead; front corners at each splay; ahead and back; left and right.
    assertThat(upToTwo).hasSize((1 + SPLAYS.size() + 2) * HEIGHTS.size() * TILTS.size());
    assertThat(upToTwo).allMatch(candidate -> candidate.cameras().size() <= 2);
  }

  @Test
  void aRobotWithoutCamerasHasNoSensorToTry() {
    RobotConfig blind =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .cameras(List.of())
                    .build())
            .build();

    assertThatThrownBy(() -> Candidates.of(blind)).hasMessageContaining("has no cameras");
  }

  @Test
  void aLayoutNeedsANameAndACamera() {
    assertThatThrownBy(() -> new Candidate(" ", List.of()))
        .hasMessageContaining("name: must not be blank")
        .hasMessageContaining("cameras: must have a camera");
  }
}
