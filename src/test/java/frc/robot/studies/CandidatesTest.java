package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.groups.Tuple.tuple;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import frc.robot.RobotConfig;
import frc.robot.RobotConfigBuilder;
import frc.robot.Robots;
import frc.robot.lib.config.BodyBuilder;
import frc.robot.lib.config.CameraConfig;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

/** The layouts the study tries. */
class CandidatesTest {
  @Test
  void theRobotsOwnComesFirstThenTheFamily() {
    List<Candidate> candidates = Candidates.of(Robots.SIM);

    assertThat(candidates.get(0).cameras()).isEqualTo(Robots.SIM.cameras());
    // 13 layouts of one to four cameras, at 3 heights and 4 tilts.
    assertThat(candidates).hasSize(1 + 13 * 3 * 4);
    Set<String> names = new HashSet<>();
    for (Candidate candidate : candidates) {
      assertThat(names.add(candidate.name())).as(candidate.name()).isTrue();
      candidate.checkOn(Robots.SIM); // within the robot, or it throws
      assertThat(candidate.cameras())
          .extracting(CameraConfig::sensor)
          .containsOnly(Robots.SIM.cameras().get(0).sensor());
    }
  }

  @Test
  void theFamilyFacesOutFromTheFrame() {
    Candidate corners =
        Candidates.of(Robots.SIM).stream()
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
        RobotConfigBuilder.builder(Robots.SIM)
            .body(BodyBuilder.builder(Robots.SIM.body()).height(Inches.of(20)).build())
            .cameras(List.of(StudyFixtures.FRONT))
            .build();

    assertThat(Candidates.of(shortRobot)).hasSize(1 + 13 * 2 * 4);
  }

  @Test
  void aRobotWithoutCamerasHasNoSensorToTry() {
    RobotConfig blind = RobotConfigBuilder.builder(Robots.SIM).cameras(List.of()).build();

    assertThatThrownBy(() -> Candidates.of(blind)).hasMessageContaining("has no cameras");
  }

  @Test
  void aLayoutNeedsANameAndACamera() {
    assertThatThrownBy(() -> new Candidate(" ", List.of()))
        .hasMessageContaining("name: must not be blank")
        .hasMessageContaining("cameras: must have a camera");
  }
}
