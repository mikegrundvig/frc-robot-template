package frc.robot.lib.config;

import static frc.robot.lib.config.ConfigFixtures.body;
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
    var length = Inches.of(28);
    var width = Inches.of(24);
    var height = Inches.of(30);
    var bumper = Inches.of(3.5);
    var mass = Kilograms.of(60);
    var inertia = KilogramSquareMeters.of(6);
    var cg = Meters.of(0.2);
    var zero = Inches.of(0);

    assertRejected("frameLength", () -> new Body(zero, width, height, bumper, mass, inertia, cg));
    assertRejected("frameWidth", () -> new Body(length, zero, height, bumper, mass, inertia, cg));
    assertRejected("height", () -> new Body(length, width, zero, bumper, mass, inertia, cg));
    assertRejected(
        "bumperThickness", () -> new Body(length, width, height, zero, mass, inertia, cg));
    assertRejected(
        "mass", () -> new Body(length, width, height, bumper, Kilograms.of(-1), inertia, cg));
    assertRejected(
        "yawMomentOfInertia",
        () ->
            new Body(length, width, height, bumper, mass, KilogramSquareMeters.of(Double.NaN), cg));
    assertRejected("cgHeight", () -> new Body(length, width, height, bumper, mass, inertia, zero));
  }

  @Test
  void centerOfGravityMustBeBelowTheTop() {
    assertRejected(
        "cgHeight",
        () ->
            new Body(
                Inches.of(28),
                Inches.of(24),
                Inches.of(30),
                Inches.of(3.5),
                Kilograms.of(60),
                KilogramSquareMeters.of(6),
                Inches.of(30)));
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

  private static Translation3d inches(double x, double y, double z) {
    return new Translation3d(Inches.of(x), Inches.of(y), Inches.of(z));
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }
}
