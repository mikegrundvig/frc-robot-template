package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.camera;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.sensor;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

class CameraConfigTest {
  @Test
  void keepsItsDescription() {
    CameraConfig camera = camera("front", new Translation3d(0.3, 0.2, 0.25));

    assertThat(camera.name()).isEqualTo("front");
    assertThat(camera.forward().in(Meters)).isCloseTo(0.3, within(1e-9));
    assertThat(camera.left().in(Meters)).isCloseTo(0.2, within(1e-9));
    assertThat(camera.up().in(Meters)).isCloseTo(0.25, within(1e-9));
    assertThat(camera.roll().in(Degrees)).isZero();
    assertThat(camera.tiltUp().in(Degrees)).isZero();
    assertThat(camera.yaw().in(Degrees)).isZero();
    assertThat(camera.sensor()).isEqualTo(sensor());

    CameraConfig.Sensor sensor = camera.sensor();
    assertThat(sensor.widthPixels()).isEqualTo(1280);
    assertThat(sensor.heightPixels()).isEqualTo(800);
    assertThat(sensor.diagonalFieldOfView().in(Degrees)).isCloseTo(80, within(1e-9));
    assertThat(sensor.frameRate().in(Hertz)).isCloseTo(30, within(1e-9));
    assertThat(sensor.latency().in(Milliseconds)).isCloseTo(30, within(1e-9));
  }

  @Test
  void aCameraTiltedUpHasANegativePitchInWpilibTerms() {
    CameraConfig camera =
        CameraConfigBuilder.builder(camera("front-left", new Translation3d()))
            .forward(Inches.of(11.5))
            .left(Inches.of(11.5))
            .up(Inches.of(9))
            .tiltUp(Degrees.of(20))
            .yaw(Degrees.of(45))
            .build();

    assertThat(camera.position())
        .isEqualTo(new Translation3d(Inches.of(11.5), Inches.of(11.5), Inches.of(9)));
    assertThat(camera.robotToCamera())
        .isEqualTo(
            new Transform3d(
                camera.position(), new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(45))));
  }

  @Test
  void theMountMustBeNumbers() {
    var nan = Double.NaN;

    assertRejected("CameraConfig[ ].name", () -> change().name(" ").build());
    assertRejected("CameraConfig[front].forward", () -> change().forward(Inches.of(nan)).build());
    assertRejected("CameraConfig[front].left", () -> change().left(Inches.of(nan)).build());
    assertRejected("CameraConfig[front].up", () -> change().up(Inches.of(nan)).build());
    assertRejected("CameraConfig[front].roll", () -> change().roll(Degrees.of(nan)).build());
    assertRejected("CameraConfig[front].tiltUp", () -> change().tiltUp(Degrees.of(nan)).build());
    assertRejected("CameraConfig[front].yaw", () -> change().yaw(Degrees.of(nan)).build());
  }

  @Test
  void aSensorWithNoLatencyIsAllowed() {
    assertThatNoException().isThrownBy(() -> changeSensor().latency(Milliseconds.of(0)).build());
  }

  @Test
  void rejectsAnImpossibleSensor() {
    assertRejected("Sensor.widthPixels", () -> changeSensor().widthPixels(0).build());
    assertRejected("Sensor.heightPixels", () -> changeSensor().heightPixels(-800).build());
    assertRejected(
        "Sensor.diagonalFieldOfView",
        () -> changeSensor().diagonalFieldOfView(Degrees.of(0)).build());
    assertRejected(
        "Sensor.diagonalFieldOfView: 180 ° must be below 180 °",
        () -> changeSensor().diagonalFieldOfView(Degrees.of(180)).build());
    assertRejected("Sensor.frameRate", () -> changeSensor().frameRate(Hertz.of(0)).build());
    assertRejected("Sensor.latency", () -> changeSensor().latency(Milliseconds.of(-1)).build());
  }

  /** A copy of a valid camera named "front", to change one value. */
  private static CameraConfigBuilder change() {
    return CameraConfigBuilder.builder(camera("front", new Translation3d()));
  }

  private static CameraConfigSensorBuilder changeSensor() {
    return CameraConfigSensorBuilder.builder(sensor());
  }

  private static void assertRejected(String message, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(message);
  }
}
