package frc.robot.sim;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.assumed;
import static frc.robot.subsystems.mechanisms.JointFixtures.control;
import static frc.robot.subsystems.mechanisms.JointFixtures.profile;
import static frc.robot.subsystems.mechanisms.JointFixtures.setup;
import static frc.robot.subsystems.mechanisms.JointFixtures.soft;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.robot.Joint;
import frc.robot.lib.phoenix.MechanismControl;
import frc.robot.lib.phoenix.MechanismControlBuilder;
import frc.robot.subsystems.mechanisms.JointIO;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * One simulated joint, its motor controller and its physics, from the fixtures' robot: gravity as
 * the model works it out, against the feedforward the model seeds; its hard stops; neutral.
 */
class JointIOSimTest {
  private static final double STEP = 0.02;

  /** The pivot's seeded gravity feedforward: the volts that hold it level. */
  private static final double KG = ROBOT.math().idealGains(PIVOT).kG();

  private static JointIOSim joint(Joint joint, MechanismControl control) {
    return new JointIOSim(ROBOT.math(), setup(joint, assumed(control)));
  }

  /** Runs one joint alone for a time, gravity worked out where it is at each step. */
  private static void run(JointIOSim sim, double seconds) {
    for (int i = 0; i < Math.round(seconds / STEP); i++) {
      double gravity =
          -ROBOT.math().gravityTorque(sim.joint(), Map.of(sim.joint(), sim.position()));
      sim.step(STEP, gravity);
    }
  }

  private static MechanismControl pivotControl(Optional<Double> kG) {
    MechanismControl seeded = control(30, 1, profile(8, 30));
    return MechanismControl.builder()
        .gains(
            MechanismControl.Gains.builder()
                .kP(30)
                .kI(0)
                .kD(1)
                .kS(0)
                .kV(Optional.empty())
                .kA(Optional.empty())
                .kG(kG)
                .build())
        .softLimits(seeded.softLimits())
        .profile(seeded.profile())
        .currentLimits(seeded.currentLimits())
        .neutralMode(seeded.neutralMode())
        .bands(seeded.bands())
        .homing(seeded.homing())
        .build();
  }

  @Test
  void itStartsStillWhereItsDefinitionRestsIt() {
    JointIOSim sim = joint(PIVOT, pivotControl(Optional.empty()));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    sim.updateInputs(inputs);

    assertThat(inputs.connected).isTrue();
    assertThat(inputs.position).isEqualTo(PIVOT.reference());
    assertThat(inputs.velocity).isZero();
    assertThat(inputs.unconfigured).isEmpty();
  }

  /**
   * Held at 45° on its seeded gravity feedforward, the pivot stays there: the model's gravity and
   * its feedforward agree. Without it, the pivot sags until its feedback makes up gravity's volts.
   */
  @Test
  void theSeededGravityFeedforwardHoldsWhatTheModelsGravityPulls() {
    JointIOSim seeded = joint(PIVOT, pivotControl(Optional.empty()));
    JointIOSim none = joint(PIVOT, pivotControl(Optional.of(0.0)));
    double target = Math.toRadians(45);
    seeded.place(target);
    none.place(target);

    seeded.setPosition(target, 0, 0);
    none.setPosition(target, 0, 0);
    run(seeded, 2);
    run(none, 2);

    assertThat(KG).as("2 kg 0.3 m out takes volts to hold").isGreaterThan(0.1);
    assertThat(seeded.position()).isCloseTo(target, within(1e-4));
    assertThat(target - none.position())
        .as("its sag: the volts gravity takes, over kP")
        .isCloseTo(KG * Math.cos(target) / 30, within(0.02 * KG / 30));
  }

  /** Coasting, the pivot falls under its weight to its lower stop, and stays on it. */
  @Test
  void coastingItFallsToItsStopAndStaysThere() {
    MechanismControl coast =
        MechanismControlBuilder.builder(pivotControl(Optional.empty()))
            .neutralMode(MechanismControl.NeutralMode.COAST)
            .build();
    JointIOSim sim = joint(PIVOT, coast);
    sim.place(Math.toRadians(60));

    sim.stop();
    run(sim, 2);

    assertThat(sim.position()).isEqualTo(0);
    assertThat(sim.velocity()).isZero();
  }

  /** Braking, its motors' back EMF slows the fall: after the same 0.3 s, it's fallen less. */
  @Test
  void brakingItFallsSlowerThanCoasting() {
    JointIOSim brake = joint(PIVOT, pivotControl(Optional.empty()));
    JointIOSim coast =
        joint(
            PIVOT,
            MechanismControlBuilder.builder(pivotControl(Optional.empty()))
                .neutralMode(MechanismControl.NeutralMode.COAST)
                .build());
    brake.place(Math.toRadians(60));
    coast.place(Math.toRadians(60));

    brake.stop();
    coast.stop();
    run(brake, 0.3);
    run(coast, 0.3);

    assertThat(brake.position()).isLessThan(Math.toRadians(60));
    assertThat(brake.position()).isGreaterThan(coast.position());
  }

  /** Full voltage up: the elevator stops driving at its soft limit, short of its hard stop. */
  @Test
  void itDrivesNoFurtherThanItsSoftLimits() {
    JointIOSim sim = joint(ELEVATOR, control(60, 2, soft(0, 1.15), profile(1.2, 5), 40));

    sim.setVoltage(12);
    double highest = 0;
    for (int i = 0; i < 100; i++) {
      run(sim, STEP);
      highest = Math.max(highest, sim.position());
    }

    assertThat(highest).isLessThan(1.2);
    assertThat(sim.position()).isCloseTo(1.15, within(0.03));
  }

  /** A duty cycle is a share of the battery's 12 V: half of it, as 6 V does. */
  @Test
  void aDutyCycleIsAShareOfTheBattery() {
    JointIOSim duty = joint(ELEVATOR, control(60, 2, profile(1.2, 5)));
    JointIOSim volts = joint(ELEVATOR, control(60, 2, profile(1.2, 5)));

    duty.setDutyCycle(0.5);
    volts.setVoltage(6);
    run(duty, 0.5);
    run(volts, 0.5);

    assertThat(duty.position()).isEqualTo(volts.position());
    assertThat(duty.position()).isGreaterThan(0.1);
  }

  /**
   * Its stator limit holds each motor: each of the elevator's two draws 20 A, as its leader reads.
   */
  @Test
  void itsStatorLimitHoldsEachMotor() {
    JointIOSim sim = joint(ELEVATOR, control(60, 2, Optional.empty(), profile(1.2, 5), 20));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    sim.setVoltage(12);
    sim.step(0.001, 0);
    sim.updateInputs(inputs);

    assertThat(inputs.statorCurrentAmps).isCloseTo(20, within(1e-6));
  }
}
