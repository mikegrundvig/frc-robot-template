package frc.robot.lib.robot;

import static frc.robot.lib.robot.ConfigFixtures.body;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

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
    assertThat(body.bumperThickness().in(Inches)).isCloseTo(3.5, within(1e-9));
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
  }

  @Test
  void everyMeasurementMustBePositive() {
    var zero = Inches.of(0);

    assertRejected("frameLength", () -> change().frameLength(zero).build());
    assertRejected("frameWidth", () -> change().frameWidth(zero).build());
    assertRejected("height", () -> change().height(zero).build());
    assertRejected("bumperThickness", () -> change().bumperThickness(zero).build());
    assertRejected("mass", () -> change().mass(Kilograms.of(-1)).build());
    assertRejected(
        "yawMomentOfInertia",
        () -> change().yawMomentOfInertia(KilogramSquareMeters.of(Double.NaN)).build());
    assertRejected("cgHeight", () -> change().cgHeight(zero).build());
  }

  @Test
  void everyBadMeasurementIsReportedAtOnce() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                change()
                    .frameLength(Inches.of(0))
                    .mass(Kilograms.of(-1))
                    .cgHeight(Inches.of(Double.NaN))
                    .build())
        .withMessage(
            "Body.frameLength: 0 in must be positive\n"
                + "Body.mass: -1 Kg must be positive\n"
                + "Body.cgHeight: NaN in must be a finite number");
  }

  @Test
  void centerOfGravityMustBeBelowTheTop() {
    assertRejected("cgHeight", () -> change().cgHeight(Inches.of(30)).build());
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
