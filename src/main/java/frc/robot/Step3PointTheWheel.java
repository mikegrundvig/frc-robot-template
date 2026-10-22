package frc.robot;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.driverstation.XboxController;
import org.wpilib.framework.TimedRobot;

/**
 * Step 3: point the wheel. The steering motor turns the front left wheel to an angle and holds it
 * there, watching the CANcoder to see where the wheel is. Press A to point it straight ahead and B
 * to turn it 90 degrees.
 */
public class Step3PointTheWheel extends TimedRobot {
  // Not checked on this chassis yet: check them in Phoenix Tuner X.
  static final String CANIVORE_NAME = "canivore";
  static final int STEER_MOTOR_ID = 24;
  static final int CANCODER_ID = 12;
  static final double ZERO = 0.10693359375; // the CANcoder's magnet offset, in turns
  static final double STEER_GEAR_RATIO = 287.0 / 11.0; // motor turns for one turn of the wheel

  CANBus canivore = new CANBus(CANIVORE_NAME);
  TalonFX steerMotor = new TalonFX(STEER_MOTOR_ID, canivore);
  CANcoder canCoder = new CANcoder(CANCODER_ID, canivore);
  XboxController gamepad = new XboxController(0);

  public Step3PointTheWheel() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = ZERO;
    canCoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.withRemoteCANcoder(canCoder); // the motor steers by the CANcoder's angle
    config.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;
    config.ClosedLoopGeneral.ContinuousWrap = true; // turn the short way round
    config.Slot0.kP = 20; // volts for each turn it's off: small, so it moves gently
    config.CurrentLimits.StatorCurrentLimit = 20; // amps: weak, so a hand can stop it
    steerMotor.getConfigurator().apply(config);
  }

  @Override
  public void teleopPeriodic() {
    if (gamepad.getAButton()) {
      steerMotor.setControl(new PositionVoltage(Degrees.of(0)));
    }
    if (gamepad.getBButton()) {
      steerMotor.setControl(new PositionVoltage(Degrees.of(90)));
    }
  }
}
