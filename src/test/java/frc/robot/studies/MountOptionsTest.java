package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.SENSOR;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import frc.robot.Robots;
import frc.robot.lib.config.CameraConfig;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** CAD's places for cameras, and every layout made from them. */
class MountOptionsTest {
  @TempDir Path folder;

  private Path file(String... lines) throws IOException {
    return Files.write(folder.resolve("mounts.csv"), List.of(lines));
  }

  /** Three places, a camera there facing 3, 2, and 1 ways. */
  private MountOptions three() throws IOException {
    return MountOptions.read(
        file(
            "# name, forward, left, up, yaws, tilts",
            "front-left, 12, 12, 20, 0 30 45, 10",
            "",
            "back-right, -12, -12, 20, 180 -150, 10",
            "top, 0, 0, 28, 0, 15"));
  }

  @Test
  void everyLayoutOfTheirPlacesFacingEveryWay() throws IOException {
    MountOptions places = three();

    // One camera: 3 + 2 + 1. Two: 3 x 2 + 3 x 1 + 2 x 1. Three: 3 x 2 x 1.
    assertThat(places.layouts(1)).isEqualTo(6);
    assertThat(places.layouts(2)).isEqualTo(17);
    assertThat(places.layouts(3)).isEqualTo(23);
    List<Candidate> layouts = places.candidates(Robots.SIM, SENSOR, 3);
    assertThat(layouts).hasSize(23);
    Set<String> names = new HashSet<>();
    for (Candidate layout : layouts) {
      assertThat(names.add(layout.name())).as(layout.name()).isTrue();
      layout.checkOn(Robots.SIM);
    }
    assertThat(places.candidates(Robots.SIM, SENSOR, 2)).hasSize(17);
  }

  @Test
  void aLayoutsCamerasAreItsPlacesFacingTheirWays() throws IOException {
    Candidate layout =
        three().candidates(Robots.SIM, SENSOR, 2).stream()
            .filter(
                candidate ->
                    candidate.name().equals("2 cameras: front-left 30°, up 10°; top 0°, up 15°"))
            .findFirst()
            .orElseThrow();

    CameraConfig frontLeft = layout.cameras().get(0);
    assertThat(frontLeft.name()).isEqualTo("front-left");
    assertThat(frontLeft.forward().in(Inches)).isCloseTo(12, within(1e-9));
    assertThat(frontLeft.up().in(Inches)).isCloseTo(20, within(1e-9));
    assertThat(frontLeft.yaw().in(Degrees)).isCloseTo(30, within(1e-9));
    assertThat(frontLeft.tiltUp().in(Degrees)).isCloseTo(10, within(1e-9));
    assertThat(layout.cameras().get(1).name()).isEqualTo("top");
  }

  @Test
  void aPlaceOutsideTheRobotIsNamed() throws IOException {
    MountOptions places = MountOptions.read(file("pole, 40, 0, 20, 0, 10"));

    assertThatThrownBy(() -> places.candidates(Robots.SIM, SENSOR, 1))
        .hasMessageContaining("the place pole")
        .hasMessageContaining("outside the robot");
  }

  @Test
  void tooManyLayoutsSaysHowMany() throws IOException {
    String ways = "0 10 20 30 40 50 60 70, 0 10 20 30";
    MountOptions places =
        MountOptions.read(
            file(
                "a, 10, 10, 20, " + ways,
                "b, 10, -10, 20, " + ways,
                "c, -10, 10, 20, " + ways,
                "d, -10, -10, 20, " + ways));

    assertThatThrownBy(() -> places.candidates(Robots.SIM, SENSOR, 4))
        .hasMessageContaining("make 1,185,920 layouts of 1 to 4 cameras")
        .hasMessageContaining("more than the 5,000")
        .hasMessageContaining("--cameras=3");
  }

  @Test
  void aBadLineIsNamed() throws IOException {
    assertThatThrownBy(() -> MountOptions.read(file("a, 10, 10, 20, 0")))
        .hasMessageContaining("mounts.csv line 1")
        .hasMessageContaining("6 values");
    assertThatThrownBy(() -> MountOptions.read(file("# places", "a, 10, ten, 20, 0, 10")))
        .hasMessageContaining("line 2");
    assertThatThrownBy(() -> MountOptions.read(file("a, 10, 10, 20, , 95")))
        .hasMessageContaining("Option[a].yawsDegrees: must have at least one")
        .hasMessageContaining("Option[a].tiltsDegrees: [0] 95 must be from -90 to 90");
    assertThatThrownBy(() -> MountOptions.read(file("a, 10, 10, 20, 0, 10", "a, 0, 0, 20, 0, 10")))
        .hasMessageContaining("two places are named a");
    assertThatThrownBy(() -> MountOptions.read(file("# nothing yet")))
        .hasMessageContaining("has no places");
  }
}
