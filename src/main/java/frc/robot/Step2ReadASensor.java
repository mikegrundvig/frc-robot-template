package frc.robot;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import org.wpilib.framework.TimedRobot;
import org.wpilib.telemetry.Telemetry;

/**
 * Step 2: read a sensor. The CANcoder on the front left module measures which way the wheel
 * points, and this prints that angle twice a second. No need to enable: turn the wheel by hand and
 * watch the number. Pointing straight ahead, it should read close to 0.
 */
public class Step2ReadASensor extends TimedRobot {
  // Not checked on this chassis yet: check them in Phoenix Tuner X.
  static final String CANIVORE_NAME = "canivore";
  static final int CANCODER_ID = 12;
  static final double ZERO = 0.10693359375; // the magnet offset, in turns: straight ahead reads 0

  CANcoder canCoder = new CANcoder(CANCODER_ID, new CANBus(CANIVORE_NAME));
  int loops = 0;

  public Step2ReadASensor() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = ZERO;
    canCoder.getConfigurator().apply(config);
  }

  @Override
  public void robotPeriodic() {
    double degrees = canCoder.getAbsolutePosition().getValue().in(Degrees);
    Telemetry.log("Wheel angle (degrees)", degrees); // in the simulator window, under Telemetry
    loops = loops + 1;
    if (loops % 25 == 0) { // this runs 50 times a second, so this prints twice a second
      System.out.println("Wheel angle: " + Math.round(degrees) + " degrees");
    }
  }
}
