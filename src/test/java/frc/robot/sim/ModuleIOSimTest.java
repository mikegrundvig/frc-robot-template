package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveConfigLimitsBuilder;
import frc.robot.subsystems.drive.ModuleIO;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation2d;

/** A simulated module's steering answers to its configuration, as a motor controller does. */
@ExtendWith(RobotSim.class)
class ModuleIOSimTest {
  private static final SwerveConfig DRIVETRAIN = Robots.SIM.drivetrain();

  /** The simulator's step: a fifth of a 20 ms loop. */
  private static final double STEP_SECONDS = 0.004;

  /** A module as configured, with the steering's current limit tuned to this. */
  private static ModuleIOSim module(double steerLimitAmps) {
    ModuleIOSim module = new ModuleIOSim(DRIVETRAIN.modules(), 0, 0);
    module.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());
    module.configure(
        DRIVETRAIN.gains(),
        SwerveConfigLimitsBuilder.builder(DRIVETRAIN.limits())
            .steerStatorCurrent(Amps.of(steerLimitAmps))
            .build());
    return module;
  }

  /** Steps the module, told to turn a quarter turn, until it has turned this far: the seconds. */
  private static double secondsToTurn(ModuleIOSim module, double degrees) {
    module.setSteerAngle(Rotation2d.CCW_90DEG);
    double seconds = 0;
    while (module.trueSteerAngle().getDegrees() < degrees) {
      module.step(STEP_SECONDS);
      seconds += STEP_SECONDS;
      assertThat(seconds).as("turned within a second").isLessThan(1);
    }
    return seconds;
  }

  /**
   * The steering's current limit comes from the configuration, as tuned live: a lower one gets the
   * module under way more slowly, since the motor gives less torque while it speeds up.
   */
  @Test
  void aLowerSteeringCurrentLimitGetsTheModuleTurningMoreSlowly() {
    double atForty = secondsToTurn(module(40), 10);
    double atTen = secondsToTurn(module(10), 10);

    assertThat(atTen).as("%s s at 10 A, %s s at 40 A", atTen, atForty).isGreaterThan(atForty * 1.5);
  }

  /**
   * The steering's current, as the module reports it, is the most since the last loop: the limit in
   * the loop that starts a turn, though by that loop's end the motor is at speed and draws next to
   * nothing.
   */
  @Test
  void theSteeringReportsItsPeakCurrentWhichStaysWithinItsLimit() {
    ModuleIOSim module = module(20);
    ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
    module.setSteerAngle(Rotation2d.CCW_90DEG);

    for (int loop = 0; loop < 25; loop++) {
      for (int step = 0; step < 5; step++) {
        module.step(STEP_SECONDS);
      }
      module.updateInputs(inputs);
      assertThat(inputs.steerStatorCurrentAmps).isLessThanOrEqualTo(20);
      if (loop == 0) {
        assertThat(inputs.steerStatorCurrentAmps).as("the turn's first loop").isEqualTo(20);
      }
    }
    assertThat(inputs.steerStatorCurrentAmps).as("turned and holding").isLessThan(1);
  }
}
