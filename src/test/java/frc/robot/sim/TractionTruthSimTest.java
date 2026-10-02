package frc.robot.sim;

import static frc.robot.sim.KnownGap.Check.stillOpen;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.ImuReadings;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.SimulationTest;
import frc.robot.subsystems.drive.ModuleIO;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The truth suite's traction: where the tread runs out of grip, against Coulomb friction (μ times
 * the load on each wheel) and the loads that quasi-static weight transfer gives. Today's simulator
 * caps the robot as a whole at μ g, every wheel alike, so the limit is right in every direction but
 * no wheel slips before the others.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class TractionTruthSimTest {
  /** The template's tread: 1.2. */
  private static final double MU =
      Robots.SIM.definition().drivetrain().modules().wheelCoefficientOfFriction();

  /**
   * Pushed past its grip (120 A against μ 1.2) in each of 16 directions, the robot speeds up at μ
   * g, the most its tread holds with every wheel slipping, whatever the direction: within 3% of μ
   * g, and every direction within 5% of the others. A friction pyramid (Coulomb friction along and
   * across each wheel, apart) would give up to 41% more at 45°.
   */
  @Test
  void pastItsGripTheRobotSpeedsUpAtMuGInEveryDirection() {
    List<Double> accelerations = new ArrayList<>();
    for (int k = 0; k < 16; k++) {
      Rotation2d direction = Rotation2d.fromDegrees(22.5 * k);
      TruthRig rig = new TruthRig();
      rig.steer(direction);
      rig.run(0.5);
      rig.limitDrives(120);
      rig.driveVolts(12);
      rig.run(0.06);
      Translation2d early = rig.velocity();
      rig.run(0.1);
      Translation2d change = rig.velocity().minus(early);

      accelerations.add(change.getNorm() / 0.1);
      assertThat(change.getAngle().orElseThrow().minus(direction).getDegrees())
          .as("the way the wheels point, at %s", direction)
          .isCloseTo(0, within(0.5));
    }

    double gripLimit = MU * ImuReadings.STANDARD_GRAVITY;
    double least = accelerations.stream().mapToDouble(Double::doubleValue).min().orElseThrow();
    double most = accelerations.stream().mapToDouble(Double::doubleValue).max().orElseThrow();
    assertThat(accelerations)
        .allSatisfy(a -> assertThat(a).isCloseTo(gripLimit, within(0.03 * gripLimit)));
    assertThat(most / least).isLessThan(1.05);
  }

  /**
   * From rest, the drive motors pushing straight ahead at a steady current: the least current at
   * which each wheel slips, found by halving between runs, puts each wheel's push at μ times its
   * own load. Speeding up shifts load to the rear wheels (m a h / L off each front wheel, h the
   * center of gravity's height and L the wheelbase), so the front wheels slip first, at μ N within
   * 3%: about 47 A on the template's robot, where its register says.
   *
   * <p>Today every wheel slips at once, at the push that would speed the whole robot up at μ g,
   * since the simulator caps the robot as a whole and loads every wheel alike.
   */
  @Test
  @KnownGap("the robot as a whole is held to μ g; no wheel carries a load of its own")
  void eachWheelSlipsWhereItsPushPassesMuTimesItsLoad() {
    TruthRig template = new TruthRig();
    double[] onsets = new double[4];
    for (int wheel = 0; wheel < 4; wheel++) {
      double gripping = 10;
      double slipping = 150;
      while (slipping - gripping > 0.05) {
        double amps = (gripping + slipping) / 2;
        if (slips(amps, wheel)) {
          slipping = amps;
        } else {
          gripping = amps;
        }
      }
      onsets[wheel] = template.force(slipping);
    }

    double m = template.mass();
    double g = ImuReadings.STANDARD_GRAVITY;
    double h = template.body.cgHeight().in(Meters);
    double wheelbase = template.wheels[0].getX() - template.wheels[2].getX();
    double front = MU * m * g / 4 / (1 + 2 * MU * m * h / (wheelbase * template.effectiveMass()));
    boolean allAtOnce = Math.abs(onsets[0] - onsets[3]) < 0.01 * onsets[0];
    stillOpen("the front left wheel's push at its slip, N", onsets[0], front, 0.03, 208.9, 0.5);
    stillOpen(
        "the front wheels slip first",
        onsets[0] < onsets[2] && onsets[1] < onsets[3],
        allAtOnce,
        "the wheels' pushes at their slips are " + java.util.Arrays.toString(onsets) + " N");
  }

  /**
   * Speeding up straight ahead at the template's 30 A, gripping, the wheels carry the robot's
   * weight as quasi-static weight transfer shares it: each front wheel m g / 4 − m a h / (2 L),
   * each rear one as much more, within 3%, with a the robot's acceleration, h the center of
   * gravity's height, and L the wheelbase. Today every wheel carries a quarter of the weight
   * however hard the robot speeds up.
   */
  @Test
  @KnownGap("every wheel carries a quarter of the robot's weight")
  void speedingUpShiftsLoadFromTheFrontWheelsToTheRear() {
    TruthRig rig = new TruthRig();
    rig.driveVolts(12);
    rig.run(0.1);
    double before = rig.velocity().getX();
    rig.loops(1);
    double acceleration = (rig.velocity().getX() - before) / 0.02;

    double m = rig.mass();
    double h = rig.body.cgHeight().in(Meters);
    double wheelbase = rig.wheels[0].getX() - rig.wheels[2].getX();
    double front = m * ImuReadings.STANDARD_GRAVITY / 4 - m * acceleration * h / (2 * wheelbase);
    WheelTruth frontLeft = rig.simulation.physics().wheel(0);
    WheelTruth backLeft = rig.simulation.physics().wheel(2);
    assertThat(acceleration).as("speeding up").isGreaterThan(4);
    stillOpen("the front left wheel's load, N", frontLeft.loadNewtons(), front, 0.03, 166.25, 0.01);
    assertThat(backLeft.loadNewtons())
        .as("today, the back's too")
        .isEqualTo(frontLeft.loadNewtons());
  }

  /**
   * One module pointing 10° off the others (its absolute encoder zeroed off, so its steering holds
   * it there), all four driven ahead at 1 m/s: the three that agree hold the robot straight, their
   * grip across their wheels (3 μN) far past the one wheel's scrub (μN), and that wheel slides. So
   * physics says the robot neither drifts sideways nor turns: under 1 mm and 0.05° in a second.
   *
   * <p>Today the robot moves as the best fit of all four wheels, so wheels that disagree move it
   * where none of them is going: it drifts toward the odd wheel and turns, about 5° a second, and
   * since odometry fits the wheels the same way, it never errs for it.
   */
  @Test
  @KnownGap("the robot moves as the best fit of its wheels, so wheels that disagree can't scrub")
  void threeWheelsThatAgreeHoldTheRobotStraightAgainstOneThatDoesnt() {
    TruthRig rig =
        new TruthRig(Robots.SIM.definition().body(), MU, SensorErrors.NONE.withSteerOffset(0, 10));
    double kv = Robots.SIM.drivetrain().gains().driveKv();
    rig.steer(Rotation2d.ZERO);
    rig.run(0.5);
    for (ModuleIO module : rig.modules) {
      module.setDriveVelocity(1.0 / rig.wheelRadius(), kv);
    }
    rig.run(0.5);

    Pose2d before = rig.simulation.truePose();
    rig.run(1);
    Pose2d after = rig.simulation.truePose();
    double sideways = after.relativeTo(before).getY();
    double turned = after.getRotation().minus(before.getRotation()).getDegrees();

    stillOpen(
        "the robot held straight by the three wheels that agree",
        Math.abs(turned) < 0.05 && Math.abs(sideways) < 0.001,
        Math.abs(turned - 4.79) < 0.05,
        "it turned " + turned + "° and drifted " + sideways + " m in a second");
  }

  /**
   * Whether this wheel slips within 0.2 s of a launch from rest at this current: its tread moving
   * over the floor faster than a millimeter a second, against the floor under it.
   */
  private static boolean slips(double amps, int wheel) {
    TruthRig rig = new TruthRig();
    rig.limitDrives(amps);
    rig.driveVolts(12);
    for (int loop = 0; loop < 10; loop++) {
      double before = rig.simulation.physics().wheel(wheel).rotationRadians();
      Translation2d start = rig.simulation.truePose().getTranslation();
      rig.loops(1);
      double rolled =
          (rig.simulation.physics().wheel(wheel).rotationRadians() - before) * rig.wheelRadius();
      double moved = rig.simulation.truePose().getTranslation().getDistance(start);
      if ((rolled - moved) / 0.02 > 0.001) {
        return true;
      }
    }
    return false;
  }
}
