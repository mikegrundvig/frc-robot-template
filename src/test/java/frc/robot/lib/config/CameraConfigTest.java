package frc.robot.lib.config;

import static frc.robot.lib.config.ConfigFixtures.camera;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Milliseconds;

import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

class CameraConfigTest {
  @Test
  void keepsItsDescription() {
    CameraConfig camera = camera("front", new Translation3d(0.3, 0.2, 0.25));

    assertThat(camera.name()).isEqualTo("front");
    assertThat(camera.robotToCamera().getTranslation())
        .isEqualTo(new Translation3d(0.3, 0.2, 0.25));
    assertThat(camera.widthPixels()).isEqualTo(1280);
    assertThat(camera.heightPixels()).isEqualTo(800);
    assertThat(camera.diagonalFieldOfView().in(Degrees)).isCloseTo(80, within(1e-9));
    assertThat(camera.frameRate().in(Hertz)).isCloseTo(30, within(1e-9));
    assertThat(camera.latency().in(Milliseconds)).isCloseTo(30, within(1e-9));
  }

  @Test
  void aCameraWithNoLatencyIsAllowed() {
    assertThatNoException()
        .isThrownBy(() -> cameraWith("ideal", 1280, 800, Degrees.of(80), Hertz.of(30), ms(0)));
  }

  @Test
  void rejectsImpossibleValues() {
    var fov = Degrees.of(80);
    var rate = Hertz.of(30);

    assertRejected("camera name", () -> cameraWith(" ", 1280, 800, fov, rate, ms(30)));
    assertRejected("front widthPixels", () -> cameraWith("front", 0, 800, fov, rate, ms(30)));
    assertRejected("front heightPixels", () -> cameraWith("front", 1280, -800, fov, rate, ms(30)));
    assertRejected(
        "front diagonalFieldOfView",
        () -> cameraWith("front", 1280, 800, Degrees.of(0), rate, ms(30)));
    assertRejected(
        "front diagonalFieldOfView must be under 180",
        () -> cameraWith("front", 1280, 800, Degrees.of(180), rate, ms(30)));
    assertRejected(
        "front frameRate", () -> cameraWith("front", 1280, 800, fov, Hertz.of(0), ms(30)));
    assertRejected("front latency", () -> cameraWith("front", 1280, 800, fov, rate, ms(-1)));
  }

  private static CameraConfig cameraWith(
      String name, int width, int height, Angle fov, Frequency rate, Time latency) {
    return new CameraConfig(name, new Transform3d(), width, height, fov, rate, latency);
  }

  private static Time ms(double milliseconds) {
    return Milliseconds.of(milliseconds);
  }

  private static void assertRejected(String message, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(message);
  }
}
