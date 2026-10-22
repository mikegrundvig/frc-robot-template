package frc.robot;

import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.driverstation.XboxController;
import org.wpilib.framework.TimedRobot;

/**
 * Step 4: one whole swerve module. Point the left stick any way: the front left wheel turns to
 * point that way, and the further you push, the faster it spins (20% power at most). Let go and
 * it stops.
 */
public class Step4OneModule extends TimedRobot {
  // Not checked on this chassis yet: check them in Phoenix Tuner X.
  static final String CANIVORE_NAME = "canivore";
  static final int DRIVE_MOTOR_ID = 23;
  static final int STEER_MOTOR_ID = 24;
  static final int CANCODER_ID = 12;
  static final double ZERO = 0.10693359375; // the CANcoder's magnet offset, in turns
  static final double STEER_GEAR_RATIO = 287.0 / 11.0; // motor turns for one turn of the wheel

  CANBus canivore = new CANBus(CANIVORE_NAME);
  TalonFX driveMotor = new TalonFX(DRIVE_MOTOR_ID, canivore);
  TalonFX steerMotor = new TalonFX(STEER_MOTOR_ID, canivore);
  CANcoder canCoder = new CANcoder(CANCODER_ID, canivore);
  XboxController gamepad = new XboxController(0);

  public Step4OneModule() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = ZERO;
    canCoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration config = new TalonFXConfiguration(); // the steering, as in step 3
    config.Feedback.withRemoteCANcoder(canCoder);
    config.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;
    config.ClosedLoopGeneral.ContinuousWrap = true;
    config.Slot0.kP = 20;
    config.CurrentLimits.StatorCurrentLimit = 20;
    steerMotor.getConfigurator().apply(config);
  }

  @Override
  public void teleopPeriodic() {
    double forward = -gamepad.getLeftY(); // pushed forward reads negative, so flip it
    double left = -gamepad.getLeftX(); // pushed left reads negative, so flip it
    double push = Math.min(Math.hypot(forward, left), 1); // how far the stick is pushed, 0 to 1
    if (push > 0.1) {
      double angle = Math.atan2(left, forward); // the stick's direction: 0 is straight ahead
      steerMotor.setControl(new PositionVoltage(Radians.of(angle)));
      driveMotor.setControl(new DutyCycleOut(push * 0.2)); // 20% at most
    } else {
      driveMotor.setControl(new DutyCycleOut(0)); // let go: stop, and leave the wheel pointing
    }
  }
}
