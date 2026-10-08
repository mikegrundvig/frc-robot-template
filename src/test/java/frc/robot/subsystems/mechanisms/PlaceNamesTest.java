package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Where a joint is, in its places' names: the pivot's in at 1.5°, run at 110°, out at 130°. */
class PlaceNamesTest {
  private static final PlaceNames PIVOT_NAMES = new PlaceNames(PIVOT, Math.toRadians(2));

  private static String at(double degrees) {
    return PIVOT_NAMES.at(Math.toRadians(degrees));
  }

  @Test
  void itsAtAPlaceWithinItsBand() {
    assertThat(at(1.5)).isEqualTo("at in");
    assertThat(at(108.5)).isEqualTo("at run");
    assertThat(at(131.9)).isEqualTo("at out");
  }

  @Test
  void itsBetweenTwoOrBeyondThem() {
    assertThat(at(60)).isEqualTo("between in and run");
    assertThat(at(120)).isEqualTo("between run and out");
    assertThat(at(-3)).isEqualTo("below in");
    assertThat(at(134)).isEqualTo("above out");
  }

  /** A joint with no places reads as a number. */
  @Test
  void aJointWithNoPlacesReadsAsANumber() {
    PlaceNames none =
        new PlaceNames(
            JointRevoluteBuilder.builder(TURRET).positions(List.of()).build(), Math.toRadians(2));

    assertThat(none.at(Math.toRadians(45))).isEqualTo("45°");
  }
}
