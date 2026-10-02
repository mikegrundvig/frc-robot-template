package frc.robot.subsystems.coprocessors;

import org.littletonrobotics.junction.AutoLog;

/**
 * A button on the robot that powers the coprocessors down, as Pitwall's action does: a switch on
 * one of Systemcore's digital inputs ({@link PowerButtonIODigitalInput}), or none, the default.
 */
public interface PowerButtonIO extends AutoCloseable {
  @AutoLog
  class PowerButtonIOInputs {
    /** Whether the button is pressed now. */
    public boolean pressed = false;
  }

  default void updateInputs(PowerButtonIOInputs inputs) {}

  @Override
  default void close() {}
}
