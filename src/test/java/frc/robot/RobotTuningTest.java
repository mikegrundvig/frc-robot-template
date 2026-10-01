package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.pitwall.ConfigRules;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.pitwall.TunableTee;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/** Every robot's configuration works with live tuning, within the limits its mechanisms declare. */
class RobotTuningTest {
  /**
   * Declaring a mechanism's tuning fails if a committed value is outside its limits; setting each
   * value to what it already is rebuilds the configuration through its checks, and changes nothing.
   */
  @Test
  void everyRobotsCommittedValuesCanBeDialedIn() {
    for (RobotConfig robot : Robots.all()) {
      for (LiveConfig<?> live : RobotContainer.tunings(robot).all()) {
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

  /** The container's list of tunings is the one every check reads: it misses none of them. */
  @Test
  void theListOfTuningsHasEveryOne() {
    var tunings = RobotContainer.tunings(Robots.SIM);

    assertThat(tunings.all())
        .hasSize(RobotContainer.Tunings.class.getRecordComponents().length)
        .doesNotHaveDuplicates()
        .extracting(LiveConfig::name)
        .doesNotHaveDuplicates();
  }

  @Test
  void aGainIsTunedByItsPathWithinItsLimits() {
    var drivetrain = Drive.tuning(Robots.SIM.drivetrain());

    assertThat(drivetrain.set(Map.of("gains.steerKp", 18.5))).isTrue();
    assertThat(drivetrain.get().gains().steerKp()).isEqualTo(18.5);
    assertThat(drivetrain.whyNot(Map.of("gains.steerKp", 500.0)))
        .isEqualTo("drivetrain: gains.steerKp: 500 is above its limit of 100");
  }

  /**
   * The page says what each tunable value means, from its record's Javadoc, which the build writes
   * into the program as it compiles: every one needs a {@code @param} saying what it does.
   */
  @Test
  void everyTunableValueSaysWhatItMeans() {
    for (LiveConfig<?> live : RobotContainer.tunings(Robots.SIM).all()) {
      for (LiveConfig.Knob knob : live.knobs()) {
        String component = knob.path().substring(knob.path().lastIndexOf('.') + 1);
        assertThat(live.meaning(knob.path()))
            .as(
                "%s %s says nothing: add @param %s to %s's Javadoc, saying what it does and in"
                    + " what unit",
                live.name(),
                knob.path(),
                component,
                live.documentation().value(knob.path()).record())
            .isNotBlank();
      }
    }
  }

  /** Within its limits isn't enough: the records' own checks have the last word, and say which. */
  @Test
  void aValueTheRecordsRejectIsRefusedNamingTheRecordsValue() {
    var vision = Vision.tuning(Robots.SIM.vision());

    assertThat(vision.whyNot(Map.of("maxAmbiguity", 0.0)))
        .startsWith("vision: VisionConfig.maxAmbiguity: 0 ");
  }

  /**
   * The page shows a refusal beside the value it concerns. A record's message names the value the
   * way {@code Checks} writes it, from the record that checked it ({@code
   * VisionConfig.maxAmbiguity}), and Pitwall reads that back to the value's tunable.
   */
  @Test
  void aRecordsRefusalIsTiedToTheTunableItNames() {
    var tee = new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false);
    TunableRegistry.registerBackend("", tee);
    try {
      TunableRegistry.publish("Vision/config", Vision.tuning(Robots.SIM.vision()));
      tee.update();

      var outcome = tee.requestChanges(Map.of("/Vision/config/maxAmbiguity", 0.0));
      tee.update();

      assertThat(outcome.getNow(null).reasons())
          .singleElement()
          .satisfies(
              reason -> {
                assertThat(reason.layer()).isEqualTo(Outcome.Layer.RECORD);
                assertThat(reason.by()).isEqualTo("VisionConfig");
                assertThat(reason.paths()).containsExactly("/Vision/config/maxAmbiguity");
              });
    } finally {
      TunableRegistry.reset();
    }
  }

  /**
   * The drivetrain's top speed is tuned live, but never past what its motors and gearing can reach,
   * which the robot's definition says and the pit can't change: the robot's rules hold it there,
   * and the page shows why beside the value.
   */
  @Test
  void theTopSpeedIsHeldToTheDrivetrainsFreeSpeed() {
    var drivetrain = Drive.tuning(Robots.SIM.drivetrain());
    RobotRules.register(
        new ConfigRules(), Robots.SIM.definition(), drivetrain, Vision.tuning(Robots.SIM.vision()));

    assertThat(drivetrain.set(Map.of("limits.maxLinearSpeed", 4.5))).isTrue();
    assertThat(drivetrain.whyNot(Map.of("limits.maxLinearSpeed", 6.0)))
        .contains("6 m/s is faster than the drivetrain's free speed of 4.729 m/s");
    assertThat(drivetrain.set(Map.of("limits.maxLinearSpeed", 6.0))).isFalse();
    assertThat(drivetrain.get().limits().maxLinearSpeed().in(MetersPerSecond)).isEqualTo(4.5);
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
            "mountToleranceMeters",
            "latencyMarginSeconds");
    assertThat(vision.set(Map.of("maxAmbiguity", 0.1))).isTrue();
    assertThat(vision.get().maxAmbiguity()).isEqualTo(0.1);
  }
}
