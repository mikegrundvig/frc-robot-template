package frc.robot.lib.config;

import static frc.robot.lib.config.ConfigFixtures.camera;
import static frc.robot.lib.config.ConfigFixtures.swerve;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

class ConfigTunerTest {
  @Test
  void findsEveryTunableValueByPath() {
    var tuner = new ConfigTuner<>(swerve());

    assertThat(tuner.knobs())
        .extracting(ConfigTuner.Knob::path)
        .containsExactly(
            "limits.driveStatorCurrent",
            "limits.driveSupplyCurrent",
            "limits.steerStatorCurrent",
            "limits.maxLinearSpeed",
            "limits.maxAngularSpeed",
            "gains.driveKs",
            "gains.driveKv",
            "gains.driveKp",
            "gains.steerKp",
            "gains.steerKd",
            "gains.skewLookahead");
    assertThat(tuner.knobs().get(0).unit()).as("in the committed unit").isEqualTo(Amps.symbol());
    assertThat(tuner.knobs().get(5).unit()).as("a plain number").isEmpty();
  }

  @Test
  void startsAtTheCommittedValues() {
    var tuner = new ConfigTuner<>(swerve());

    assertThat(tuner.current()).isEqualTo(tuner.committed()).isEqualTo(swerve());
    assertThat(tuner.value("gains.steerKp")).isEqualTo(16.0);
    assertThat(tuner.value("limits.driveStatorCurrent")).isCloseTo(40, within(1e-9));
    assertThat(tuner.changes()).isEmpty();
  }

  @Test
  void aTunedValueRebuildsTheConfiguration() {
    var tuner = new ConfigTuner<>(swerve());

    tuner.set("gains.steerKp", 18.5);
    tuner.set("limits.driveStatorCurrent", 35);

    assertThat(tuner.current().gains().steerKp()).isEqualTo(18.5);
    assertThat(tuner.current().limits().driveStatorCurrent().in(Amps)).isCloseTo(35, within(1e-9));
    assertThat(tuner.current().modules()).as("the rest is untouched").isEqualTo(swerve().modules());
    assertThat(tuner.committed()).isEqualTo(swerve());
    assertThat(tuner.value("gains.steerKp")).isEqualTo(18.5);
  }

  @Test
  void aRejectedValueLeavesTheConfigurationAsItWas() {
    var tuner = new ConfigTuner<>(swerve());
    tuner.set("gains.steerKp", 18.5);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> tuner.set("gains.steerKd", -1))
        .withMessageStartingWith("steerKd");
    // Checked across records too: the drivetrain can't be told to outrun its motors.
    assertThatIllegalArgumentException()
        .isThrownBy(() -> tuner.set("limits.maxLinearSpeed", 10))
        .withMessageContaining("free speed");

    assertThat(tuner.current().gains().steerKp()).isEqualTo(18.5);
    assertThat(tuner.current().gains().steerKd()).isEqualTo(0.08);
    assertThat(tuner.value("limits.maxLinearSpeed")).isCloseTo(4, within(1e-9));
  }

  @Test
  void listsWhatDiffersWithTheBuilderCallThatCommitsIt() {
    var tuner = new ConfigTuner<>(swerve());

    tuner.set("gains.steerKp", 18.5);
    tuner.set("limits.driveStatorCurrent", 35);
    tuner.set("limits.maxAngularSpeed", 1.5);
    tuner.set("gains.skewLookahead", 0.05);

    String amps = " " + Amps.symbol();
    String rotationsPerSecond = " " + RotationsPerSecond.symbol();
    String seconds = " " + Seconds.symbol();
    assertThat(tuner.changes())
        .containsExactly(
            new ConfigTuner.Change(
                "limits.driveStatorCurrent",
                "40.0" + amps,
                "35.0" + amps,
                ".driveStatorCurrent(Amps.of(35.0))"),
            new ConfigTuner.Change(
                "limits.maxAngularSpeed",
                "1.0" + rotationsPerSecond,
                "1.5" + rotationsPerSecond,
                ".maxAngularSpeed(RotationsPerSecond.of(1.5))"),
            new ConfigTuner.Change("gains.steerKp", "16.0", "18.5", ".steerKp(18.5)"),
            new ConfigTuner.Change(
                "gains.skewLookahead",
                "0.03" + seconds,
                "0.05" + seconds,
                ".skewLookahead(Seconds.of(0.05))"));
  }

  @Test
  void settingTheCommittedValueAgainIsNoChange() {
    var tuner = new ConfigTuner<>(swerve());

    tuner.set("gains.steerKp", 18.5);
    tuner.set("gains.steerKp", 16.0);

    assertThat(tuner.changes()).isEmpty();
  }

  @Test
  void resetReturnsToTheCommittedConfiguration() {
    var tuner = new ConfigTuner<>(swerve());
    tuner.set("gains.steerKp", 18.5);

    tuner.reset();

    assertThat(tuner.current()).isEqualTo(swerve());
    assertThat(tuner.value("gains.steerKp")).isEqualTo(16.0);
    assertThat(tuner.changes()).isEmpty();
  }

  @Test
  void onlyTunableValuesCanBeTuned() {
    var tuner = new ConfigTuner<>(swerve());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> tuner.set("gains.steerKi", 1))
        .withMessage("no tunable value at gains.steerKi");
    assertThatIllegalArgumentException()
        .as("a fact about the hardware")
        .isThrownBy(() -> tuner.value("modules.driveGearing"))
        .withMessage("no tunable value at modules.driveGearing");
  }

  @Test
  void aConfigurationWithNothingTunableHasNoKnobs() {
    assertThat(new ConfigTuner<>(camera("front", new Translation3d())).knobs()).isEmpty();
  }

  /** Only numbers and measures can be dialed in. */
  record Named(@Tunable String name) {}

  @Test
  void onlyNumbersAndMeasuresCanBeMarkedTunable() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new ConfigTuner<>(new Named("front")))
        .withMessageStartingWith("name is marked @Tunable");
  }
}
