package frc.robot.sim;

import static frc.robot.sim.KnownGap.Check.stillOpen;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Terrain;
import frc.robot.RobotSim;
import frc.robot.SimulationTest;
import frc.robot.subsystems.drive.ModuleIO;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The truth suite's dynamics: the simulated robot against Newton, with nothing of the simulator's
 * own earlier output as the answer. Force against mass, torque against the yaw inertia, momentum
 * and energy kept or spent as they must be, and a robot that stands still standing still. What
 * today's simulator fails is written as a {@link KnownGap}, asserting today's number.
 *
 * <p>Each test commands the motor controllers directly ({@link TruthRig}), as the robot's code
 * would, and reads the truth the simulator keeps; none reads a sensor.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DynamicsTruthSimTest {
  /**
   * A tread that can't slip at 60 A, even with the front wheels lightened by weight transfer (they
   * would at the template's 1.2): this checks force and mass, not grip.
   */
  private static final double NO_SLIP = 2.0;

  /**
   * From rest, every drive motor held at its stator limit pushes the robot with 4 Kt I G / r, and
   * physics says it speeds up at that over the robot's mass, each wheel's own moving parts counted
   * as mass at its tread: m + 4 J / r². Within 1% after the first 50 ms (the steering and the first
   * step settle), at 30 and at 60 A, while the motors stay at their limit (300 ms).
   *
   * <p>Today the robot speeds up about 2% short of it at either current: the motor controller's
   * limit is applied once a 4 ms step, as a voltage the drive's motor model then holds through the
   * step, and the current falls a little within each step as the wheel speeds up
   * (CurrentLimitedMotorSim's notes describe the same for the steering, which they fix there).
   */
  @ParameterizedTest
  @ValueSource(doubles = {30, 60})
  @KnownGap("the drive's current limit holds only at the start of each 4 ms step")
  void atTheStatorLimitTheRobotAcceleratesAsForceOverMass(double amps) {
    TruthRig rig = new TruthRig(NO_SLIP);
    rig.limitDrives(amps);
    rig.driveVolts(12);

    rig.run(0.06);
    double early = rig.velocity().getX();
    rig.run(0.24);
    double measured = (rig.velocity().getX() - early) / 0.24;

    double physics = 4 * rig.force(amps) / rig.effectiveMass();
    stillOpen(
        "the acceleration at " + amps + " A, m/s²",
        measured,
        physics,
        0.01,
        amps == 30 ? 4.3529 : 8.7058,
        0.002);
    assertThat(rig.velocity().getY()).as("straight ahead").isCloseTo(0, within(1e-9));
  }

  /**
   * Each module steered across its arm and pushed at a current well below slip, the robot spins up
   * as the four wheels' torque about its center over its yaw inertia, the wheels' own spin counted:
   * α = 4 F d / (I + 4 J d² / r²). Read back as the yaw inertia that acceleration implies, it's the
   * configured one ({@code Body.yawMomentOfInertia}) within 2%.
   *
   * <p>Today the robot spins as if all its mass sat at its wheels, m d² (each wheel carries a
   * quarter of the mass as inertia on its motor), and the configured inertia is read by nothing.
   */
  @Test
  @KnownGap("each wheel carries a quarter of the mass, so the robot spins as if it sat at them")
  void pushedAroundItsCenterTheRobotSpinsAsItsYawInertiaSays() {
    TruthRig rig = new TruthRig();
    rig.steerTangent();
    rig.run(0.5); // the modules turn to their angles
    rig.limitDrives(10);
    rig.driveVolts(12);

    rig.run(0.06);
    double early = rig.simulation.yawRateRadiansPerSecond();
    rig.run(0.24);
    double measured = (rig.simulation.yawRateRadiansPerSecond() - early) / 0.24;

    double d = rig.arm();
    double r = rig.wheelRadius();
    double wheels = 4 * rig.wheelInertia() * d * d / (r * r);
    double implied = 4 * rig.force(10) * d / measured - wheels;
    double configured = rig.body.yawMomentOfInertia().in(KilogramSquareMeters);
    stillOpen("the yaw inertia the robot spins as, kg m²", implied, configured, 0.02, 11.085, 0.01);
    assertThat(rig.simulation.truePose().getTranslation().getNorm())
        .as("spinning in place")
        .isLessThan(1e-9);
  }

  /** Spinning with its drive motors free (friction off), it keeps its rate within 0.1% for 2 s. */
  @Test
  void spinningFreeTheRobotKeepsItsRate() {
    TruthRig rig = new TruthRig();
    rig.steerTangent();
    rig.run(0.5);
    rig.limitDrives(30);
    rig.driveVolts(12);
    rig.run(0.5);
    rig.limitDrives(FREE);

    rig.run(0.04);
    double before = rig.simulation.yawRateRadiansPerSecond();
    rig.run(2);

    assertThat(before).as("spun up").isGreaterThan(2);
    assertThat(rig.simulation.yawRateRadiansPerSecond()).isCloseTo(before, within(before * 1e-3));
  }

  /**
   * A drive motor whose controller lets no current through: the wheel turns freely, as with
   * friction off. The configuration needs a positive limit; this one is a nanoampere.
   */
  private static final double FREE = 1e-9;

  /** Coasting with friction off, the robot keeps its momentum within 0.1% over 5 s. */
  @Test
  void coastingFreeTheRobotKeepsItsMomentum() {
    TruthRig rig = new TruthRig();
    rig.steer(Rotation2d.fromDegrees(30));
    rig.run(0.5);
    rig.driveVolts(12);
    rig.run(0.3);
    rig.limitDrives(FREE);

    rig.run(0.04);
    Translation2d before = rig.velocity().times(rig.mass());
    rig.run(5);
    Translation2d after = rig.velocity().times(rig.mass());

    assertThat(before.getNorm()).as("launched").isGreaterThan(50);
    assertThat(after.getDistance(before)).isLessThan(before.getNorm() * 1e-3);
  }

  /**
   * With its drive motors switched off (braking, as a motor at 0 V does), the robot's kinetic
   * energy never rises: by no more than 0.01 J in any loop, where a step's rounding is far below
   * that.
   */
  @Test
  void brakingTheRobotNeverGainsEnergy() {
    TruthRig rig = new TruthRig();
    rig.steer(Rotation2d.fromDegrees(-20));
    rig.run(0.5);
    rig.driveVolts(12);
    rig.run(0.6);
    rig.driveVolts(0);

    double before = kineticEnergy(rig);
    double mostGained = Double.NEGATIVE_INFINITY;
    for (int loop = 0; loop < 100; loop++) {
      rig.loops(1);
      double now = kineticEnergy(rig);
      mostGained = Math.max(mostGained, now - before);
      before = now;
    }

    assertThat(mostGained).isLessThan(0.01);
    assertThat(before).as("stopped").isLessThan(0.01);
  }

  /**
   * Coasting with friction off, its wheels steered a quarter turn: physics says the robot's energy
   * can only fall, as the wheels scrub sideways over the carpet until they roll its way. Today the
   * wheels keep their spin whatever the floor does, and the robot turns to follow them, its
   * velocity dipping and recovering: its energy falls, then rises back almost to what it was.
   */
  @Test
  @KnownGap("the floor never drags a wheel's spin: the robot follows its wheels within its grip")
  void steeringCoastingWheelsNeverGainsEnergy() {
    TruthRig rig = new TruthRig();
    rig.run(0.2);
    rig.driveVolts(12);
    rig.run(0.4);
    rig.limitDrives(FREE);
    rig.run(0.1);
    rig.steer(Rotation2d.CCW_90DEG);

    double lowest = kineticEnergy(rig);
    double rise = 0;
    for (int loop = 0; loop < 50; loop++) {
      rig.loops(1);
      double now = kineticEnergy(rig);
      lowest = Math.min(lowest, now);
      rise = Math.max(rise, now - lowest);
    }

    stillOpen(
        "energy never rises while coasting",
        rise <= 0.01,
        Math.abs(rise - 35.15) < 0.5,
        "it rose " + rise + " J after its lowest");
  }

  /**
   * A launch at the stator limit, wheels gripping: the motors' work (their torque, Kt I G, through
   * each wheel's turn) is the robot's kinetic energy, its wheels' spin included, within 2%. Today
   * it's 3.5% short: 2.2% as the acceleration is ({@link
   * #atTheStatorLimitTheRobotAcceleratesAsForceOverMass}), the motors giving less than their limit
   * between the starts of steps; and the rest because the robot moves each step at its wheels'
   * average speed over it, half a step behind their spin, so at 0.3 s it carries 1.3% less energy
   * than wheels and robot moving together would.
   */
  @Test
  @KnownGap("the drive's current limit holds only at the start of each 4 ms step")
  void aGrippingLaunchTurnsTheMotorsWorkIntoKineticEnergy() {
    TruthRig rig = new TruthRig();
    double[] start = wheelTurns(rig);
    rig.driveVolts(12);
    rig.run(0.3);

    double work = motorWork(rig, start, rig.force(30) * rig.wheelRadius());
    double energy = kineticEnergy(rig);
    stillOpen("kinetic energy over the motors' work", energy / work, 1, 0.02, 0.9648, 0.001);
  }

  /**
   * A launch past the tread's grip (120 A against μ 1.2): the motors' work is the robot's kinetic
   * energy plus what the slipping wheels lose to friction, μ N times how far each slips, within 2%.
   * Today each wheel carries its share of the robot's inertia while it slips, so the motors' work
   * goes into a robot the floor holds back, and a third of it goes nowhere.
   */
  @Test
  @KnownGap("a slipping wheel still carries its share of the robot's inertia")
  void aSlippingLaunchSpendsTheMotorsWorkOnEnergyAndFriction() {
    TruthRig rig = new TruthRig();
    double[] start = wheelTurns(rig);
    rig.limitDrives(120);
    rig.driveVolts(12);
    rig.run(0.14);

    double work = motorWork(rig, start, rig.force(120) * rig.wheelRadius());
    double r = rig.wheelRadius();
    double[] end = wheelTurns(rig);
    double traveled = rig.simulation.truePose().getX();
    double load = rig.mass() * ImuReadings.STANDARD_GRAVITY / 4;
    double friction = 0;
    for (int i = 0; i < 4; i++) {
      friction += 1.2 * load * ((end[i] - start[i]) * r - traveled);
    }
    double spent = kineticEnergy(rig) + friction;

    assertThat(friction).as("the wheels slipped").isPositive();
    stillOpen(
        "kinetic energy and friction over the motors' work", spent / work, 1, 0.02, 0.6553, 0.002);
  }

  /** Standing for a minute with its motors off, the robot creeps less than 1 mm and 0.01°. */
  @Test
  void standingStillTheRobotStaysPut() {
    TruthRig rig = new TruthRig();
    Pose2d start = new Pose2d(3, 4, Rotation2d.fromDegrees(30));
    rig.simulation.place(start);
    rig.driveVolts(0);

    rig.run(60);

    assertCreptLessThanAMillimeter(rig, start);
  }

  /**
   * Braked on a 15° ramp, the robot creeps less than 1 mm and 0.01° in a minute: its tread (μ 1.2)
   * holds far more than the slope's pull (tan 15°, 0.27 of its weight). Today's simulator feels no
   * pull down the slope at all, so it passes for want of a force; an engine that feels gravity must
   * pass it on its brakes and its grip.
   */
  @Test
  void brakedOnARampTheRobotStaysPut() {
    double height = 1.0;
    Bump ramp =
        new Bump(
            "A 15° ramp",
            new Translation2d(5, -1),
            new Translation2d(5, 9),
            height,
            2 * height / Math.tan(Math.toRadians(15)));
    TruthRig rig = new TruthRig();
    rig.simulation.setTerrain(new Terrain(List.of(ramp)));
    Pose2d start = new Pose2d(3.3, 4, Rotation2d.ZERO);
    rig.simulation.place(start);
    rig.driveVolts(0);

    rig.run(60);

    assertThat(Math.toDegrees(-rig.simulation.pitchRadians()))
        .as("nose up the ramp")
        .isCloseTo(15, within(1e-6));
    assertCreptLessThanAMillimeter(rig, start);
  }

  private static void assertCreptLessThanAMillimeter(TruthRig rig, Pose2d start) {
    Pose2d end = rig.simulation.truePose();
    assertThat(end.getTranslation().getDistance(start.getTranslation())).isLessThan(0.001);
    assertThat(Math.abs(end.getRotation().minus(start.getRotation()).getDegrees()))
        .isLessThan(0.01);
  }

  /** Every wheel's turn since the start, in radians: what it rolled, slipping or not. */
  private static double[] wheelTurns(TruthRig rig) {
    double[] turns = new double[4];
    for (int i = 0; i < 4; i++) {
      turns[i] = rig.simulation.moduleSim(i).wheelRotationRadians();
    }
    return turns;
  }

  /** The motors' work at a steady torque at the wheel: that torque through each wheel's turn. */
  private static double motorWork(TruthRig rig, double[] start, double torqueNewtonMeters) {
    double[] end = wheelTurns(rig);
    double work = 0;
    for (int i = 0; i < 4; i++) {
      work += torqueNewtonMeters * (end[i] - start[i]);
    }
    return work;
  }

  /**
   * The robot's kinetic energy, in joules: its mass moving, its body spinning about its center at
   * its configured yaw inertia, and each wheel's own spin.
   */
  private static double kineticEnergy(TruthRig rig) {
    double speed = rig.velocity().getNorm();
    double yawRate = rig.simulation.yawRateRadiansPerSecond();
    double energy =
        0.5 * rig.mass() * speed * speed
            + 0.5 * rig.body.yawMomentOfInertia().in(KilogramSquareMeters) * yawRate * yawRate;
    ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
    for (ModuleIO module : rig.modules) {
      module.updateInputs(inputs);
      double spin = inputs.driveVelocityRadPerSec;
      energy += 0.5 * rig.wheelInertia() * spin * spin;
    }
    return energy;
  }
}
