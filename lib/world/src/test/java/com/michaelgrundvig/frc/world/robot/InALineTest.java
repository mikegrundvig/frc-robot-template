package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Shape;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/** The common parts in a line: what each factory fills in, as its builder would be told. */
class InALineTest {
  private static final Translation3d AXLE = new Translation3d(0.25, 0, 0.32);

  @Test
  void aPivotTurnsBetweenItsStopsFromWhereItsStowed() {
    Link arm = Link.box("arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.5, 0.36));

    Joint.Revolute pivot =
        Joint.pivot(
            "pivot",
            Part.CHASSIS,
            AXLE,
            Joint.PITCH_DOWN,
            Range.degrees(0, 135),
            Degrees.of(1.5),
            arm,
            Position.degrees("out", 130));

    assertThat(pivot.origin().getTranslation()).isEqualTo(AXLE);
    assertThat(pivot.axis()).isEqualTo(Joint.PITCH_DOWN);
    assertThat(pivot.range()).contains(Range.degrees(0, 135));
    assertThat(pivot.reference()).isCloseTo(Math.toRadians(1.5), within(1e-12));
    assertThat(pivot.position("out").value()).isCloseTo(Math.toRadians(130), within(1e-12));
    assertThat(pivot.outputs()).isEmpty();
  }

  @Test
  void aRollerTurnsFreelyFromZeroAtItsOutputs() {
    Joint.Revolute roller =
        Joint.roller(
            "intake", Part.CHASSIS, AXLE, Link.unseen("roller"), Output.rpm("intaking", 2000));

    assertThat(roller.range()).isEmpty();
    assertThat(roller.reference()).isZero();
    assertThat(roller.output("intaking")).isEqualTo(Output.rpm("intaking", 2000));
    assertThat(roller.positions()).isEmpty();
  }

  @Test
  void anElevatorSlidesStraightUp() {
    Joint.Prismatic elevator =
        Joint.elevator(
            "elevator",
            Part.CHASSIS,
            AXLE,
            Range.meters(0, 1.2),
            Meters.of(0),
            Link.unseen("carriage"),
            Position.of("clear", Inches.of(20)));

    assertThat(elevator.axis()).isEqualTo(Joint.UP);
    assertThat(elevator.range()).contains(Range.meters(0, 1.2));
    assertThat(elevator.position("clear").value()).isCloseTo(0.508, within(1e-12));
  }

  @Test
  void aLinkIsUnweighedUnlessItsMassIsGiven() {
    Translation3d center = new Translation3d(0.1, 0, 0.2);
    Translation3d size = new Translation3d(0.2, 0.4, 0.05);

    Link drawn = Link.box("arm", center, size);
    Link weighed = Link.weighed("arm", Kilograms.of(3), center, size);

    assertThat(drawn.mass()).isEqualTo(MassProperties.NONE);
    assertThat(drawn.shapes()).hasSize(1);
    assertThat(((Shape.Box) drawn.shapes().get(0)).material()).isEqualTo(Material.ALUMINUM);
    assertThat(Link.unseen("arm").shapes()).isEmpty();
    assertThat(weighed.mass().mass().in(Kilograms)).isEqualTo(3);
    assertThat(weighed.mass().cg()).isEqualTo(center);
    assertThat(weighed.shapes()).isEqualTo(drawn.shapes());
  }

  @Test
  void aGearingLosesNothingUntilMeasuredAndAnAbsoluteEncoderTurnsWithItsJoint() {
    Joint.Revolute pivot =
        Joint.pivot(
            "pivot",
            Part.CHASSIS,
            AXLE,
            Joint.PITCH_UP,
            Range.degrees(0, 90),
            Degrees.of(0),
            Link.unseen("arm"));

    Sensor.Encoder encoder = Sensor.Encoder.absolute("pivot absolute", pivot, 1024);

    assertThat(Gearing.of(25)).isEqualTo(new Gearing(25, 1));
    assertThat(encoder.absolute()).isTrue();
    assertThat(encoder.countsPerTurn()).isEqualTo(1024);
    assertThat(encoder.terms()).hasSize(1);
    assertThat(encoder.terms().get(0).joint()).isEqualTo(pivot);
    assertThat(encoder.terms().get(0).ratio()).isEqualTo(1);
    assertThat(encoder.terms().get(0).pitchRadius()).isEqualTo(Optional.empty());
    assertThat(List.of(Joint.PITCH_UP, Joint.PITCH_DOWN)).doesNotContain(Joint.UP);
  }
}
