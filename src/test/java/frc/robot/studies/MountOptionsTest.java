package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.SENSOR;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import frc.robot.Robots;
import frc.robot.lib.robot.CameraConfig;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** CAD's places for cameras, and every layout made from them. */
class MountOptionsTest {
  private static final String INCHES =
      "name, forward (in), left (in), up (in), yaws (deg), tilts (deg)";

  @TempDir Path folder;

  private Path file(String... lines) throws IOException {
    return Files.write(folder.resolve("mounts.csv"), List.of(lines));
  }

  /** Three places, a camera there facing 3, 2, and 1 ways. */
  private MountOptions three() throws IOException {
    return MountOptions.read(
        file(
            "# Where cameras could go on the robot",
            INCHES,
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

  /** Asking for more cameras than there are places gives every layout, one camera to a place. */
  @Test
  void theCamerasAreAtMostThePlaces() throws IOException {
    MountOptions places = three();

    assertThat(places.layouts(10)).isEqualTo(23);
    assertThat(places.candidates(Robots.SIM, SENSOR, 10)).hasSize(23);
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

  /** The header says each column's unit: the same place in millimeters and radians is the same. */
  @Test
  void eachColumnInTheUnitItsHeaderNames() throws IOException {
    MountOptions metric =
        MountOptions.read(
            file(
                "name, forward (mm), left (cm), up (m), yaws (rad), tilts (deg)",
                "front-left, 304.8, 30.48, 0.508, 0.5235987755982988, 10"));

    CameraConfig camera = metric.candidates(Robots.SIM, SENSOR, 1).get(0).cameras().get(0);
    assertThat(camera.forward().in(Inches)).isCloseTo(12, within(1e-9));
    assertThat(camera.left().in(Inches)).isCloseTo(12, within(1e-9));
    assertThat(camera.up().in(Inches)).isCloseTo(20, within(1e-9));
    assertThat(camera.yaw().in(Degrees)).isCloseTo(30, within(1e-9));
  }

  @Test
  void aFileWithoutItsUnitsIsRefused() throws IOException {
    assertThatThrownBy(() -> MountOptions.read(file("front-left, 12, 12, 20, 0 30 45, 10")))
        .hasMessageContaining("mounts.csv line 1: the first line names the columns")
        .hasMessageContaining("name, forward (in)");
    assertThatThrownBy(
            () ->
                MountOptions.read(
                    file("name, forward (ft), left (in), up (in), yaws (deg), tilts (deg)")))
        .hasMessageContaining("line 1: column forward: \"forward (ft)\" should be forward (in),");
    assertThatThrownBy(
            () ->
                MountOptions.read(
                    file("name, forward, left (in), up (in), yaws (deg), tilts (deg)")))
        .hasMessageContaining("column forward");
  }

  /** A file in meters headed as inches puts every camera on the floor: refused, not run. */
  @Test
  void aPlaceOnTheFloorIsRefused() throws IOException {
    assertThatThrownBy(() -> MountOptions.read(file(INCHES, "front-left, 0.31, 0.31, 0.5, 0, 10")))
        .hasMessageContaining("line 2: Option[front-left].up: 0.5 in must be at least 1 in");
  }

  @Test
  void aPlaceOutsideTheRobotIsNamedInItsUnits() throws IOException {
    MountOptions places = MountOptions.read(file(INCHES, "pole, 40, 0, 20, 0, 10"));

    assertThatThrownBy(() -> places.candidates(Robots.SIM, SENSOR, 1))
        .hasMessage(
            "the place pole, 40 in ahead, 0 in left, and 20 in up, is outside the robot: within"
                + " 17.75 in ahead or behind its center, 17.75 in to either side, and up to 30 in");
  }

  /** Four places with 32 ways each: three cameras would still be too many, two fit. */
  @Test
  void tooManyLayoutsSaysHowManyAndWhatWouldFit() throws IOException {
    String ways = "0 10 20 30 40 50 60 70, 0 10 20 30";
    MountOptions places =
        MountOptions.read(
            file(
                INCHES,
                "a, 10, 10, 20, " + ways,
                "b, 10, -10, 20, " + ways,
                "c, -10, 10, 20, " + ways,
                "d, -10, -10, 20, " + ways));

    // Two cameras: 4 x 32 + 6 x 32 x 32 = 6,272, too many; one camera: 128.
    assertThatThrownBy(() -> places.candidates(Robots.SIM, SENSOR, 4))
        .hasMessageContaining("make 1,185,920 layouts of 1 to 4 cameras")
        .hasMessageContaining("more than the 5,000")
        .hasMessageContaining("or fewer cameras: --cameras=1 makes 128");
  }

  @Test
  void whenEvenOneCameraIsTooManyOnlyFewerWaysHelp() throws IOException {
    StringBuilder yaws = new StringBuilder();
    for (int yaw = -179; yaw <= 180; yaw++) {
      yaws.append(yaw).append(' ');
    }
    List<String> lines = new ArrayList<>(List.of(INCHES));
    for (int i = 0; i < 4; i++) {
      lines.add("p" + i + ", 10, 10, 20, " + yaws + ", 0 5 10 15");
    }
    MountOptions places = MountOptions.read(file(lines.toArray(String[]::new)));

    assertThatThrownBy(() -> places.candidates(Robots.SIM, SENSOR, 2))
        .hasMessageContaining("even one camera at a time makes too many");
  }

  @Test
  void aBadLineIsNamed() throws IOException {
    assertThatThrownBy(() -> MountOptions.read(file(INCHES, "a, 10, 10, 20, 0")))
        .hasMessageContaining("mounts.csv line 2")
        .hasMessageContaining("6 values");
    assertThatThrownBy(() -> MountOptions.read(file(INCHES, "# places", "a, 10, ten, 20, 0, 10")))
        .hasMessage("mounts.csv line 3: column left: \"ten\" isn't a number");
    assertThatThrownBy(() -> MountOptions.read(file(INCHES, "a, 10, 10, 20, , 95")))
        .hasMessageContaining("Option[a].yaws: must have at least one")
        .hasMessageContaining("Option[a].tilts: [0] 95 ° must be from -90 ° to 90 °");
    assertThatThrownBy(() -> MountOptions.read(file(INCHES, "a, 10, 10, 20, 0 30 30, 10 10")))
        .hasMessageContaining("Option[a].yaws: 30 ° is there twice")
        .hasMessageContaining("Option[a].tilts: 10 ° is there twice");
    assertThatThrownBy(
            () -> MountOptions.read(file(INCHES, "a, 10, 10, 20, 0, 10", "a, 0, 0, 20, 0, 10")))
        .hasMessageContaining("two places are named a");
    assertThatThrownBy(() -> MountOptions.read(file(INCHES))).hasMessageContaining("has no places");
    assertThatThrownBy(() -> MountOptions.read(file("# nothing yet")))
        .hasMessageContaining("has no places");
  }

  /** A spreadsheet's byte-order mark before the first column's name isn't part of it. */
  @Test
  void aSpreadsheetsByteOrderMarkIsIgnored() throws IOException {
    Path saved = folder.resolve("saved.csv");
    Files.writeString(saved, "﻿" + INCHES + "\nfront, 12, 0, 20, 0, 10\n", StandardCharsets.UTF_8);

    assertThat(MountOptions.read(saved).options()).hasSize(1);
  }
}
