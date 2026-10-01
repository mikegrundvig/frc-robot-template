package frc.robot.subsystems.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.simulation.DIOSim;

/** The power-down button on a digital input, pulled up: pressed while it reads false. */
@ExtendWith(RobotSim.class)
class PowerButtonIODigitalInputTest {
  @Test
  void pressedReadsFalse() {
    try (PowerButtonIODigitalInput button = new PowerButtonIODigitalInput(3)) {
      DIOSim input = new DIOSim(3);
      PowerButtonIOInputsAutoLogged inputs = new PowerButtonIOInputsAutoLogged();

      input.setValue(true);
      button.updateInputs(inputs);
      boolean released = inputs.pressed;
      input.setValue(false);
      button.updateInputs(inputs);

      assertThat(released).isFalse();
      assertThat(inputs.pressed).isTrue();
    }
  }
}
