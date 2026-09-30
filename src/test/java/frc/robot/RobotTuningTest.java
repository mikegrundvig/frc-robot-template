package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Every robot's configuration works with live tuning, within the limits its mechanisms declare. */
class RobotTuningTest {
  /**
   * Declaring a mechanism's tuning fails if a committed value is outside its limits; setting each
   * value to what it already is rebuilds the configuration through its checks, and changes nothing.
   */
  @Test
  void everyRobotsCommittedValuesCanBeDialedIn() {
    for (RobotConfig robot : Robots.all()) {
      List<LiveConfig<?>> tuning =
          List.of(Drive.tuning(robot.drivetrain()), Vision.tuning(robot.vision()));
      for (LiveConfig<?> live : tuning) {
        assertThat(live.knobs()).as(robot.name() + " " + live.name()).isNotEmpty();
        for (LiveConfig.Knob knob : live.knobs()) {
          assertThat(live.set(Map.of(knob.path(), live.committedValue(knob.path()))))
              .as(robot.name() + " " + live.name() + " " + knob.path())
              .isTrue();
        }
        assertThat(live.changes()).as(robot.name() + " " + live.name()).isEmpty();
      }
    }
  }

  @Test
  void aGainIsTunedByItsPathWithinItsLimits() {
    var drivetrain = Drive.tuning(Robots.SIM.drivetrain());

    assertThat(drivetrain.set(Map.of("gains.steerKp", 18.5))).isTrue();
    assertThat(drivetrain.get().gains().steerKp()).isEqualTo(18.5);
    assertThat(drivetrain.whyNot(Map.of("gains.steerKp", 500.0)))
        .isEqualTo("drivetrain: gains.steerKp: 500 is above its limit of 100");
  }

  /** Within its limits isn't enough: the records' own checks have the last word, and say which. */
  @Test
  void aValueTheRecordsRejectIsRefusedNamingTheRecordsValue() {
    var drivetrain = Drive.tuning(Robots.SIM.drivetrain());

    assertThat(drivetrain.whyNot(Map.of("limits.maxLinearSpeed", 6.0)))
        .startsWith(
            "drivetrain: SwerveConfig.limits.maxLinearSpeed: 6 m/s is faster than the"
                + " drivetrain's free speed");
  }

  /** Vision's thresholds are tuned like the drivetrain's gains: its configuration, live. */
  @Test
  void visionsThresholdsAreTunedThroughItsConfiguration() {
    var vision = Vision.tuning(Robots.SIM.vision());

    assertThat(vision.knobs())
        .extracting(LiveConfig.Knob::path)
        .containsExactly(
            "maxAmbiguity",
            "maxHeightErrorMeters",
            "linearStdDevBaseline",
            "angularStdDevBaseline",
            "distanceExponent",
            "mountToleranceMeters");
    assertThat(vision.set(Map.of("maxAmbiguity", 0.1))).isTrue();
    assertThat(vision.get().maxAmbiguity()).isEqualTo(0.1);
  }
}
