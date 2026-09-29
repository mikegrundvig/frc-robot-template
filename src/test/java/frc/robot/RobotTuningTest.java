package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.tuning.ConfigTuner;
import org.junit.jupiter.api.Test;

/** Every robot's configuration works with live tuning. */
class RobotTuningTest {
  @Test
  void everyTunableValueCanBeDialedIn() {
    for (RobotConfig robot : Robots.all()) {
      ConfigTuner<RobotConfig> tuner = new ConfigTuner<>(robot);
      assertThat(tuner.knobs()).as(robot.name()).isNotEmpty();

      // Setting each value to what it already is rebuilds the whole configuration through its
      // checks, and changes nothing.
      for (ConfigTuner.Knob knob : tuner.knobs()) {
        tuner.set(knob.path(), tuner.committedValue(knob.path()));
      }
      assertThat(tuner.changes()).as(robot.name()).isEmpty();
    }
  }

  @Test
  void valuesAreFoundByTheirPathInTheConfiguration() {
    ConfigTuner<RobotConfig> tuner = new ConfigTuner<>(Robots.SIM);

    tuner.set("drivetrain.gains.steerKp", 18.5);

    assertThat(tuner.current().drivetrain().gains().steerKp()).isEqualTo(18.5);
  }
}
