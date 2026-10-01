package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveConfigGainsBuilder;
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
    module.configure(
        DRIVETRAIN.gains(),
        SwerveConfigLimitsBuilder.builder(DRIVETRAIN.limits())
            .steerStatorCurrent(Amps.of(steerLimitAmps))
            .build());
    return module;
  }

  /**
   * The steering's current limit comes from the configuration, as tuned live. Held to its limit,
   * the motor gives a steady torque, so in the first 4 ms of a turn, while it's still at its limit,
   * a quarter of the limit turns the module a quarter as far.
   */
  @Test
  void aQuarterOfTheCurrentLimitTurnsTheModuleAQuarterAsFarAtFirst() {
    ModuleIOSim atForty = module(40);
    ModuleIOSim atTen = module(10);
    atForty.setSteerAngle(Rotation2d.CCW_90DEG);
    atTen.setSteerAngle(Rotation2d.CCW_90DEG);

    atForty.step(STEP_SECONDS);
    atTen.step(STEP_SECONDS);

    assertThat(atForty.trueSteerAngle().getDegrees()).isBetween(1.0, 3.0);
    assertThat(atTen.trueSteerAngle().getRadians())
        .isCloseTo(atForty.trueSteerAngle().getRadians() / 4, within(1e-12));
  }

  /**
   * The steering's loop runs on the angle the CANcoder last sent, every 10 ms. A stiff steerKp
   * corrects too far on each angle it hears, and past about 77 (at the committed steerKd, 0.08)
   * each correction overshoots by more than the error it corrected: a small error grows instead of
   * dying away. Each side of that edge, a tenth of a degree's error over 100 ms.
   */
  @Test
  void pastTheEdgeTheCanCodersWaitSetsASmallErrorGrowsInsteadOfDying() {
    assertThat(growth(70)).as("steerKp 70").isLessThan(0.5);
    assertThat(growth(85)).as("steerKp 85").isGreaterThan(2);
  }

  /**
   * How a tenth of a degree's error grows over 100 ms at this steerKp: the largest error in the
   * last 20 ms over the largest in the first 20. Small enough to stay under the current limit.
   */
  private static double growth(double steerKp) {
    ModuleIOSim module = new ModuleIOSim(DRIVETRAIN.modules(), 0, 0);
    module.configure(
        SwerveConfigGainsBuilder.builder(DRIVETRAIN.gains()).steerKp(steerKp).build(),
        DRIVETRAIN.limits());
    module.setSteerAngle(Rotation2d.fromDegrees(0.1));
    double first = 0;
    double last = 0;
    for (int step = 1; step <= 25; step++) {
      module.step(STEP_SECONDS);
      double error = Math.abs(0.1 - module.trueSteerAngle().getDegrees());
      if (step <= 5) {
        first = Math.max(first, error);
      } else if (step > 20) {
        last = Math.max(last, error);
      }
    }
    return last / first;
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
