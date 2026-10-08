package frc.robot.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import org.junit.jupiter.api.Test;

/**
 * How far apart a joint's absolute encoder and its motor's count may be, still, and how long they
 * must be apart before one is taken to have slipped.
 */
class EncoderGapTest {
  private static final long LOOP = 20_000_000L;

  private static double margin(Joint joint) {
    return EncoderGap.margin(
        ROBOT.with(joint).math(), joint, joint.driven().control().orElseThrow());
  }

  /**
   * The least margin, 5° on a pivot, or 2 cm on an elevator, whose encoder's count at the joint is
   * finer; a ten-turn potentiometer on the pivot's axle, turning a tenth of its range a turn of the
   * pivot, has counts 0.88° apart there, and eight of them, 7.0°, is the margin; a control's own
   * wins.
   */
  @Test
  void itsMarginIsTheLeastOrEightCountsAtTheJoint() {
    Joint.Revolute onCancoder =
        PIVOT.motorsOn(TALON_FX.can(21)).absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0)));
    Joint.Revolute onPot =
        PIVOT.absoluteEncoder(MechanismWiring.PortEncoder.analog(2, 10).offset(Rotations.of(0)));
    Joint.Prismatic elevator =
        ELEVATOR.absoluteEncoder(
            Driven.AbsoluteEncoder.of()
                .ratio(1.0 / 9)
                .on(MechanismWiring.PortEncoder.dutyCycle(2).offset(Rotations.of(0.3))));

    assertThat(Math.toDegrees(margin(onCancoder))).isCloseTo(5, within(1e-9));
    assertThat(Math.toDegrees(margin(onPot))).isCloseTo(8 * 3600.0 / 4096, within(1e-9));
    assertThat(margin(elevator)).isCloseTo(0.02, within(1e-12));
    assertThat(
            Math.toDegrees(
                margin(onCancoder.retuned(control -> control.withEncoderGap(Degrees.of(8))))))
        .isCloseTo(8, within(1e-9));
  }

  /**
   * Apart past the margin and a loop's travel at the joint's speed, for a quarter second, they're
   * said apart; a loop's travel more is allowed moving, a reading missing starts it over, and
   * inside the margin they agree.
   */
  @Test
  void theyreApartOnlyPastTheirMarginForAQuarterSecond() {
    EncoderGap gap = new EncoderGap(PIVOT, Math.toRadians(5));
    double off = Math.toRadians(6);

    String first = gap.apart(0, off, 0, 0);
    String before = gap.apart(12 * LOOP, off, 0, 0);
    String after = gap.apart(13 * LOOP, off, 0, 0);
    String moving = gap.apart(14 * LOOP, off, 0, 4);
    String missed = gap.apart(15 * LOOP, Double.NaN, 0, 0);
    String again = gap.apart(16 * LOOP, off, 0, 0);

    assertThat(first).isNull();
    assertThat(before).isNull();
    assertThat(after).isEqualTo("6° apart, more than 5°");
    assertThat(moving).as("6° inside 5° and a loop at 4 rad/s, 4.6°").isNull();
    assertThat(missed).isNull();
    assertThat(again).as("starting over").isNull();
    assertThat(gap.agree(Math.toRadians(4.9), 0)).isTrue();
    assertThat(gap.agree(off, 0)).isFalse();
  }
}
