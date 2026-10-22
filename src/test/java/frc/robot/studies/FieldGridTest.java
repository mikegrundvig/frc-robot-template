package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.WALL;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.BodyBuilder;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

/** The poses the study tries: a grid over the whole field, at several headings. */
class FieldGridTest {
  private static final Body BODY = ExampleRobot.SIM.definition().body();

  @Test
  void theGridCoversTheFieldEvenlyAndKeepsTheRobotOffTheWalls() {
    FieldGrid grid = new FieldGrid(WALL, BODY, 0.5, 8, Weights.NONE, 1);
    double margin = BODY.bumperLength().in(Meters) / 2;

    // 16 m less the robot's length (0.9 m), in cells of 0.5 m or less: 30.2, so 31 of them.
    assertThat(grid.columns()).isEqualTo(31);
    assertThat(grid.rows()).isEqualTo(15);
    assertThat(grid.spacingMeters()).isLessThanOrEqualTo(0.5);
    List<Sample> samples = grid.samples();
    assertThat(samples).hasSize(31 * 15 * 8);
    for (Sample sample : samples) {
      assertThat(sample.pose().getX()).isBetween(margin, 16 - margin);
      assertThat(sample.pose().getY()).isBetween(margin, 8 - margin);
      assertThat(sample.weight()).isEqualTo(1.0);
    }
    Sample first = samples.get(0);
    Sample last = samples.get(samples.size() - 1);
    assertThat(first.pose().getX() - margin)
        .isCloseTo(16 - margin - last.pose().getX(), within(1e-9));
    assertThat(samples.get(1).pose().getRotation().minus(first.pose().getRotation()).getDegrees())
        .isCloseTo(45, within(1e-9));
    assertThat(last.column()).isEqualTo(30);
    assertThat(last.row()).isEqualTo(14);
  }

  /**
   * Each position's headings start at their own fraction of the step, so they don't all line up
   * with the field's walls, or with cameras splayed 45°. The seed decides where.
   */
  @Test
  void eachPositionsHeadingsStartAtItsOwnOffset() {
    List<Sample> once = new FieldGrid(WALL, BODY, 2, 4, Weights.NONE, 1).samples();
    List<Sample> again = new FieldGrid(WALL, BODY, 2, 4, Weights.NONE, 1).samples();
    List<Sample> otherSeed = new FieldGrid(WALL, BODY, 2, 4, Weights.NONE, 2).samples();

    assertThat(again).isEqualTo(once);
    assertThat(otherSeed).isNotEqualTo(once);
    Set<Long> offsets = new HashSet<>();
    for (int i = 0; i < once.size(); i += 4) {
      double first = once.get(i).pose().getRotation().getDegrees();
      double offset = ((first % 90) + 90) % 90;
      offsets.add(Math.round(offset * 1000));
      for (int h = 1; h < 4; h++) {
        Rotation2d turn =
            once.get(i + h).pose().getRotation().minus(once.get(i).pose().getRotation());
        assertThat(turn.minus(Rotation2d.fromDegrees(90 * h)).getDegrees())
            .isCloseTo(0, within(1e-9));
      }
    }
    assertThat(offsets).as("a different start at each position").hasSize(once.size() / 4);
  }

  @Test
  void weightsFollowTheirRegions() {
    Weights hub = new Weights(List.of(new Weights.Region("Left end", 0, 0, 4, 8, 0)));

    List<Sample> samples = new FieldGrid(WALL, BODY, 1, 1, hub, 1).samples();

    assertThat(samples)
        .filteredOn(sample -> sample.weight() == 0)
        .isNotEmpty()
        .allMatch(sample -> sample.pose().getX() <= 4 || sample.pose().getX() >= 12);
  }

  @Test
  void aRobotTooBigForTheFieldIsRefused() {
    Body huge = BodyBuilder.builder(BODY).frameWidth(Meters.of(9)).build();

    assertThatThrownBy(() -> new FieldGrid(WALL, huge, 0.5, 8, Weights.NONE, 1))
        .hasMessageContaining("doesn't fit");
  }
}
