package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.body;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.physical.Inertia;
import java.util.Optional;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

class BodyTest {
  @Test
  void keepsItsMeasurements() {
    Body body = body();

    assertThat(body.frameLength().in(Inches)).isCloseTo(28, within(1e-9));
    assertThat(body.frameWidth().in(Inches)).isCloseTo(24, within(1e-9));
    assertThat(body.height().in(Inches)).isCloseTo(30, within(1e-9));
    assertThat(body.bumperEnds().in(Inches)).isCloseTo(3.5, within(1e-9));
    assertThat(body.bumperSides().in(Inches)).isCloseTo(3.5, within(1e-9));
    assertThat(body.bumperBottom().in(Inches)).isCloseTo(2.5, within(1e-9));
    assertThat(body.bumperTop().in(Inches)).isCloseTo(5.75, within(1e-9));
    assertThat(body.mass().in(Kilograms)).isCloseTo(60, within(1e-9));
    assertThat(body.yawMomentOfInertia().in(KilogramSquareMeters)).isCloseTo(6, within(1e-9));
    assertThat(body.cgHeight().in(Meters)).isCloseTo(0.2, within(1e-9));
  }

  @Test
  void derivesTheBumperOutlineAndFramePerimeter() {
    Body body = body();

    assertThat(body.bumperLength().in(Inches)).isCloseTo(35, within(1e-9));
    assertThat(body.bumperWidth().in(Inches)).isCloseTo(31, within(1e-9));
    assertThat(body.framePerimeter().in(Inches)).isCloseTo(104, within(1e-9));
    // Square bumpers around this frame, as a top team's: thicker at the sides than at the ends.
    Body square = change().bumperEnds(Inches.of(3.5)).bumperSides(Inches.of(5.5)).build();
    assertThat(square.bumperLength().in(Inches)).isCloseTo(35, within(1e-9));
    assertThat(square.bumperWidth().in(Inches)).isCloseTo(35, within(1e-9));
    assertThat(square.contains(new Translation3d(0, Inches.of(17.4).in(Meters), 0.1))).isTrue();
    assertThat(square.contains(new Translation3d(Inches.of(17.6).in(Meters), 0, 0.1))).isFalse();
  }

  @Test
  void everyMeasurementMustBePositive() {
    var zero = Inches.of(0);

    assertRejected("frameLength", () -> change().frameLength(zero).build());
    assertRejected("frameWidth", () -> change().frameWidth(zero).build());
    assertRejected("height", () -> change().height(zero).build());
    assertRejected("bumperEnds", () -> change().bumperEnds(zero).build());
    assertRejected("bumperSides", () -> change().bumperSides(zero).build());
    assertRejected("bumperBottom", () -> change().bumperBottom(Inches.of(-1)).build());
  }

  @Test
  void everyBadMeasurementIsReportedAtOnce() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> change().frameLength(Inches.of(0)).bumperTop(Inches.of(Double.NaN)).build())
        .withMessage(
            "Body.frameLength: 0 in must be positive\n"
                + "Body.bumperTop: NaN in must be a finite number");
  }

  @Test
  void theBumpersRunFromTheirBottomUpToTheirTopWithinTheRobot() {
    assertRejected("bumperTop", () -> change().bumperTop(Inches.of(2)).build());
    assertRejected("bumperTop", () -> change().bumperTop(Inches.of(31)).build());
  }

  /** Its mass properties together give the yaw inertia and the center of gravity's height. */
  @Test
  void itsMassPropertiesGiveTheYawInertiaAndTheCenterOfGravitysHeight() {
    Body body = body();

    assertThat(body.massProperties().mass().in(Kilograms)).isCloseTo(60, within(1e-9));
    assertThat(body.weight().weighed()).isTrue();
    assertThat(body.inertia()).isInstanceOf(Body.Turning.Tensor.class);
    assertThat(body.yawMomentOfInertia().in(KilogramSquareMeters))
        .isEqualTo(body.massProperties().inertia().izz());
    assertThat(body.cgHeight().in(Meters)).isEqualTo(body.massProperties().cg().getZ());
  }

  /**
   * Until CAD gives its inertia, a robot is a solid box of its mass, bumpers and height: flagged as
   * approximated, and never zero. Until it's weighed, it's what it's given as the season's most.
   */
  @Test
  void anUnmeasuredRobotIsABoxOfTheSeasonsMost() {
    Body body =
        change().weight(Body.Weight.unweighed(Pounds.of(149.5))).inertia(Body.Turning.BOX).build();
    Inertia box = Inertia.box(Pounds.of(149.5), inches(35, 31, 30));

    assertThat(body.weight().weighed()).isFalse();
    assertThat(body.mass().in(Pounds)).isEqualTo(149.5);
    assertThat(body.massProperties().inertia().izz()).isCloseTo(box.izz(), within(1e-12));
    assertThat(body.massProperties().inertia().ixx()).isCloseTo(box.ixx(), within(1e-12));
    assertThat(body.massProperties().inertia().whyNotSolid()).isNull();
  }

  /** A whole robot can't weigh nothing, nor turn freely: its drivetrain couldn't move it. */
  @Test
  void aRobotHasMassAndTakesEffortToTurn() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Body.Weight.weighed(Kilograms.of(0)))
        .withMessageStartingWith("Weight.mass: 0 Kg must be positive");
    assertRejected("inertia", () -> change().inertia(Body.Turning.tensor(Inertia.ZERO)).build());
  }

  /**
   * Its yaw alone, as PathPlanner or Choreo gives it: used as given, the box supplying pitch and
   * roll and no products of inertia. The fixture's 60 kg box, 35 x 31 x 30 in, rolls at 6.00 kg m²
   * and pitches at 6.86: a yaw must make each of the three at most the sum of the other two, so
   * neither 13 nor 0.5 can stand beside them. It's an error, never adjusted.
   */
  @Test
  void aYawAloneTakesItsPitchAndRollFromTheBox() {
    Body yawOnly = change().inertia(Body.Turning.yaw(KilogramSquareMeters.of(5))).build();
    Inertia box = Inertia.box(Kilograms.of(60), inches(35, 31, 30));
    Inertia resolved = yawOnly.massProperties().inertia();

    assertThat(yawOnly.yawMomentOfInertia().in(KilogramSquareMeters)).isEqualTo(5);
    assertThat(resolved.ixx()).isCloseTo(box.ixx(), within(1e-12));
    assertThat(resolved.iyy()).isCloseTo(box.iyy(), within(1e-12));
    assertThat(resolved.ixy() + resolved.ixz() + resolved.iyz()).isZero();
    assertThat(box.ixx()).isCloseTo(6.00, within(0.01));
    assertThat(box.iyy()).isCloseTo(6.86, within(0.01));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> change().inertia(Body.Turning.yaw(KilogramSquareMeters.of(13))).build())
        .withMessageContaining(
            "Body.inertia: a yaw of 13 kg m² can't stand beside the box's roll and pitch, 6.003 and"
                + " 6.855 kg m²: each must be at most the sum of the other two");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> change().inertia(Body.Turning.yaw(KilogramSquareMeters.of(0.5))).build())
        .withMessageContaining("Body.inertia: a yaw of 0.5 kg m²");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Body.Turning.yaw(KilogramSquareMeters.of(0)))
        .withMessageStartingWith("Yaw.yaw");
  }

  @Test
  void centerOfGravityMustBeInsideTheFrameAndBelowTheTop() {
    assertRejected("cg", () -> change().cg(new Translation3d(0, 0, 0.8)).build());
    assertRejected("cg", () -> change().cg(new Translation3d(0.4, 0, 0.2)).build());
    assertRejected("cg", () -> change().cg(new Translation3d(0, 0, 0)).build());
  }

  /**
   * A center of mass nobody gave is the frame's center, a third of the robot's height up: inside
   * any body, so it's never refused, and what its mass properties and height read.
   */
  @Test
  void aCenterOfMassNotGivenIsTheFramesCenterAThirdOfItsHeightUp() {
    Body body = change().cg(Optional.empty()).build(); // 30 in tall

    assertThat(body.cg()).isEmpty();
    assertThat(body.cgOrDefault()).isEqualTo(new Translation3d(0, 0, Inches.of(10).in(Meters)));
    assertThat(body.cgHeight().in(Inches)).isCloseTo(10, within(1e-9));
    assertThat(body.massProperties().cg()).isEqualTo(body.cgOrDefault());
    assertThat(body().cgOrDefault()).as("as given").isEqualTo(new Translation3d(0, 0, 0.2));
  }

  @Test
  void containsPointsInsideTheBumpersAndBelowTheTop() {
    Body body = body(); // a 35 x 31 in bumper outline, 30 in tall

    assertThat(body.contains(inches(0, 0, 0))).isTrue();
    assertThat(body.contains(inches(17.4, -15.4, 29.9))).isTrue();
    assertThat(body.contains(inches(17.6, 0, 10))).as("ahead of the front bumper").isFalse();
    assertThat(body.contains(inches(-17.6, 0, 10))).as("behind the back bumper").isFalse();
    assertThat(body.contains(inches(0, 15.6, 10))).as("beside the bumpers").isFalse();
    assertThat(body.contains(inches(0, 0, -0.1))).as("below the floor").isFalse();
    assertThat(body.contains(inches(0, 0, 30.1))).as("above the top").isFalse();
  }

  /** A copy of the valid body, to change one value. */
  private static BodyBuilder change() {
    return BodyBuilder.builder(body());
  }

  private static Translation3d inches(double x, double y, double z) {
    return new Translation3d(Inches.of(x), Inches.of(y), Inches.of(z));
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException()
        .isThrownBy(build)
        .withMessageStartingWith("Body." + field + ": ");
  }
}
