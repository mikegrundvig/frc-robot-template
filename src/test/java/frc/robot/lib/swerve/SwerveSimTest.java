package frc.robot.lib.swerve;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.assertj.core.api.Assertions.withinPercentage;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;

import frc.robot.lib.config.ConfigFixtures;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveConfigBuilder;
import frc.robot.lib.config.SwerveConfigLimitsBuilder;
import java.util.EnumSet;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** The motor controllers' side of the physics: what a module does with the commands it gets. */
class SwerveSimTest {
  private static final double LOOP_S = 0.02;

  /** At least what the battery can give, so the controllers apply all they have. */
  private static final double FULL_VOLTS = SwerveSim.Battery.TYPICAL.nominalVolts();

  private static SwerveSim simOf(SwerveConfig drivetrain) {
    return new SwerveSim(
        ConfigFixtures.body(), drivetrain, SwerveSim.Battery.TYPICAL, new Pose2d());
  }

  private static void driveAllOpenLoop(SwerveSim sim, double volts) {
    for (int i = 0; i < 4; i++) {
      sim.module(i).setSteerAngle(Rotation2d.ZERO);
      sim.module(i).setDriveVoltage(volts);
    }
  }

  @Test
  void openLoopVoltageRunsTheWheelsAtTheSpeedThatVoltageGives() {
    SwerveConfig drivetrain = ConfigFixtures.swerve();
    SwerveSim sim = simOf(drivetrain);
    driveAllOpenLoop(sim, 6.0);

    for (int loop = 0; loop < 100; loop++) {
      sim.update(LOOP_S, true);
    }

    double halfFreeSpeed = drivetrain.modules().freeSpeed().in(MetersPerSecond) / 2;
    assertThat(sim.trueFieldVelocities().vx).isCloseTo(halfFreeSpeed, withinPercentage(2));
    assertThat(sim.module(0).driveAppliedVolts()).isEqualTo(6.0);
  }

  /** What holds a drive motor controller back from giving more. */
  private enum Holding {
    STATOR_LIMIT,
    SUPPLY_LIMIT,
    BATTERY_VOLTAGE,
    NOTHING
  }

  private static Holding holding(
      SwerveSim.Module m, double statorLimit, double supplyLimit, double batteryVolts) {
    if (Math.abs(m.driveStatorAmps() - statorLimit) < 1e-9) {
      return Holding.STATOR_LIMIT;
    }
    if (Math.abs(Math.abs(m.driveSupplyAmps()) - supplyLimit) < 1e-9) {
      return Holding.SUPPLY_LIMIT;
    }
    if (Math.abs(Math.abs(m.driveAppliedVolts()) - batteryVolts) < 1e-9) {
      return Holding.BATTERY_VOLTAGE;
    }
    return Holding.NOTHING;
  }

  /**
   * A supply limit below the stator limit, as teams set to spare the battery and breakers. Asked
   * for everything, from a standing start to full speed, then braking and reversing, a controller
   * gives all its limits allow: at every moment the stator limit, the supply limit, or the
   * battery's full voltage holds it back. Stopping short of all three would give the simulated
   * robot less acceleration than the real one.
   */
  @Test
  void aSupplyLimitedDriveGetsAllTheCurrentItsLimitsAllow() {
    double statorLimit = 40;
    double supplyLimit = 20;
    SwerveConfig base = ConfigFixtures.swerve();
    SwerveSim sim =
        simOf(
            SwerveConfigBuilder.builder(base)
                .limits(
                    SwerveConfigLimitsBuilder.builder(base.limits())
                        .driveStatorCurrent(Amps.of(statorLimit))
                        .driveSupplyCurrent(Amps.of(supplyLimit))
                        .build())
                .build());

    Set<Holding> seen = EnumSet.noneOf(Holding.class);
    for (double volts : new double[] {FULL_VOLTS, -FULL_VOLTS}) {
      driveAllOpenLoop(sim, volts);
      for (int loop = 0; loop < 100; loop++) {
        sim.update(LOOP_S, true);
        for (int i = 0; i < 4; i++) {
          SwerveSim.Module m = sim.module(i);
          Holding h = holding(m, statorLimit, supplyLimit, sim.batteryVolts());
          String state =
              String.format(
                  "module %d at %.2f s of %+.1f V: stator %.2f A, supply %.2f A, applied %.2f V,"
                      + " battery %.2f V",
                  i,
                  (loop + 1) * LOOP_S,
                  volts,
                  m.driveStatorAmps(),
                  m.driveSupplyAmps(),
                  m.driveAppliedVolts(),
                  sim.batteryVolts());
          assertThat(Math.abs(m.driveSupplyAmps()))
              .as(state)
              .isLessThanOrEqualTo(supplyLimit + 1e-9);
          assertThat(Math.abs(m.driveAppliedVolts()))
              .as(state + ": no controller applies more than the battery has")
              .isLessThanOrEqualTo(sim.batteryVolts() + 1e-9);
          assertThat(h).as(state).isNotEqualTo(Holding.NOTHING);
          seen.add(h);
        }
      }
    }
    assertThat(seen)
        .containsExactlyInAnyOrder(
            Holding.STATOR_LIMIT, Holding.SUPPLY_LIMIT, Holding.BATTERY_VOLTAGE);
  }

  /**
   * At full output a motor controller draws more the higher the battery's voltage, and the
   * battery's voltage falls the more is drawn. Solved together they settle; stepped one after the
   * other they chase each other into a substep-by-substep oscillation, which a check once per loop
   * can miss.
   */
  @Test
  void atFullSpeedTheBatteryHoldsSteadySubstepToSubstep() {
    SwerveSim sim = simOf(ConfigFixtures.swerve());
    driveAllOpenLoop(sim, FULL_VOLTS);
    for (int loop = 0; loop < 100; loop++) {
      sim.update(LOOP_S, true);
    }

    double first = sim.batteryVolts();
    for (int substep = 0; substep < 10; substep++) {
      sim.update(SwerveSim.SUBSTEP_SEC, true);
      assertThat(sim.batteryVolts()).as("substep %d", substep).isCloseTo(first, within(1e-3));
    }
    SwerveSim.Battery battery = SwerveSim.Battery.TYPICAL;
    assertThat(sim.batteryVolts())
        .as("agrees with the current drawn from it")
        .isCloseTo(
            battery.nominalVolts() - battery.resistanceOhms() * sim.totalSupplyAmps(),
            within(1e-5));
    assertThat(sim.totalSupplyAmps())
        .as("near free speed, little more than the rest of the robot's load")
        .isLessThan(battery.otherLoadAmps() + 5);
  }

  @Test
  void statsBeforeAnyTimeHasPassedShowNoSlip() {
    assertThat(new SwerveSim.Stats().worstSlipFraction()).isZero();
  }
}
