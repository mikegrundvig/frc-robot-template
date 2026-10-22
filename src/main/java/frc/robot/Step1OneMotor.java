package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.driverstation.XboxController;
import org.wpilib.framework.TimedRobot;

/**
 * Step 1: one motor. The gamepad's left stick spins the front left wheel, at 20% power at most.
 * Push the stick forward and the wheel spins; pull it back and it spins the other way.
 */
public class Step1OneMotor extends TimedRobot {
  // Not checked on this chassis yet: check them in Phoenix Tuner X.
  static final String CANIVORE_NAME = "canivore";
  static final int DRIVE_MOTOR_ID = 23;

  TalonFX driveMotor = new TalonFX(DRIVE_MOTOR_ID, new CANBus(CANIVORE_NAME));
  XboxController gamepad = new XboxController(0);

  @Override
  public void teleopPeriodic() {
    double stick = -gamepad.getLeftY(); // pushed forward reads negative, so flip it
    double power = stick * 0.2; // 20% at most
    driveMotor.setControl(new DutyCycleOut(power));
  }
}
