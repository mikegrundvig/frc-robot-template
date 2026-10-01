package frc.robot.subsystems.coprocessors;

import org.wpilib.hardware.discrete.DigitalInput;

/**
 * A push button between one of Systemcore's digital inputs and ground: the input is pulled up, so
 * it reads false while the button is pressed, as on a roboRIO (on Systemcore's SmartIO,
 * unverified).
 */
public final class PowerButtonIODigitalInput implements PowerButtonIO {
  private final DigitalInput input;

  /**
   * @param channel the digital input it's wired to
   */
  public PowerButtonIODigitalInput(int channel) {
    input = new DigitalInput(channel);
  }

  @Override
  public void updateInputs(PowerButtonIOInputs inputs) {
    inputs.pressed = !input.get();
  }

  @Override
  public void close() {
    input.close();
  }
}
