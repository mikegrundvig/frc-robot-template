package frc.robot.sim;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_MOTOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.assumed;
import static frc.robot.subsystems.mechanisms.JointFixtures.control;
import static frc.robot.subsystems.mechanisms.JointFixtures.profile;
import static frc.robot.subsystems.mechanisms.JointFixtures.setup;
import static frc.robot.subsystems.mechanisms.JointFixtures.soft;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import frc.robot.Robots;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.subsystems.mechanisms.JointIO;
import frc.robot.subsystems.mechanisms.JointSetup;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * One simulated joint, its motor controller and its physics, from the fixtures' robot: gravity as
 * the model works it out, against the feedforward the model seeds; its hard stops; neutral. And
 * joints without mass, whose gravity is what the kG tuned in their blocks holds.
 */
class JointIOSimTest {
  private static final double STEP = 0.02;

  /** An arm with no mass, its kG tuned: 0.4 V, its weight pulling hardest at its zero. */
  private static final Joint.Revolute BARE_ARM =
      Joint.pivot("bare arm", Part.CHASSIS, new Translation3d(0.1, 0, 0.4), Joint.PITCH_UP)
          .range(Range.degrees(-120, 90), Degrees.of(0))
          .motor(Motor.KRAKEN_X60, Gearing.of(50))
          .control(MechanismControl.position(30).withKd(1).withKg(0.4));

  /** An elevator with no mass, its kG tuned: 0.5 V all along it. */
  private static final Joint.Prismatic BARE_LIFT =
      Joint.elevator("bare lift", Part.CHASSIS, new Translation3d(-0.2, 0, 0.1))
          .range(Range.meters(0, 1), Meters.of(0))
          .motor(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1))
          .control(MechanismControl.position(60).withKd(2).withKg(0.5));

  private static final RobotDefinition MASSLESS =
      RobotDefinitionBuilder.builder(Robots.SIM.definition())
          .name("massless")
          .joints(List.of(BARE_ARM, BARE_LIFT))
          .build();

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

  /**
   * Holds a massless joint at a target for 2 s in the simulator, its block's kG setting gravity on
   * it and its motor controller running a control of its own, and says where it ends.
   */
  private static double held(Joint joint, MechanismControl control, double target) {
    JointSimulation simulation = new JointSimulation(MASSLESS.with(joint).math());
    JointIO io = simulation.add(JointSetup.of(joint, control));
    simulation.place(joint, target);
    io.setPosition(target, 0, 0);
    for (int i = 0; i < 100; i++) {
      simulation.step(STEP);
    }
    return simulation.position(joint);
  }

  private static MechanismControl blocks(Joint joint) {
    return joint.driven().control().orElseThrow();
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
                .gravityAngle(Optional.empty())
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

  /**
   * A tuned kG holds an arm with no mass: the simulator's gravity is what that kG holds, so its
   * control's kG holds it where it's put. With kG 0 in its motor controller, it sags until its
   * feedback makes up the volts: kG times the cosine, over kP. Told nothing, it falls.
   */
  @Test
  void aTunedKgHoldsAnArmWithoutMass() {
    double target = Math.toRadians(30);
    MechanismControl tuned = blocks(BARE_ARM);

    double holding = held(BARE_ARM, tuned, target);
    double sagging = held(BARE_ARM, tuned.withKg(0), target);
    JointSimulation simulation = new JointSimulation(MASSLESS.math());
    simulation.add(JointSetup.of(BARE_ARM, tuned)).stop();
    simulation.place(BARE_ARM, target);
    for (int i = 0; i < 25; i++) {
      simulation.step(STEP);
    }

    assertThat(BARE_ARM.link().mass().isMassless()).isTrue();
    assertThat(holding).isCloseTo(target, within(1e-4));
    assertThat(target - sagging)
        .as("its sag: the volts gravity takes, over kP")
        .isCloseTo(0.4 * Math.cos(target) / 30, within(0.02 * 0.4 / 30));
    assertThat(simulation.position(BARE_ARM)).isLessThan(target - 0.05);
  }

  /** The same for a slide: its tuned kG holds it all along, and without it, it sags by kG / kP. */
  @Test
  void aTunedKgHoldsASlideWithoutMass() {
    MechanismControl tuned = blocks(BARE_LIFT);

    assertThat(held(BARE_LIFT, tuned, 0.5)).isCloseTo(0.5, within(1e-5));
    assertThat(0.5 - held(BARE_LIFT, tuned.withKg(0), 0.5))
        .isCloseTo(0.5 / 60, within(0.02 * 0.5 / 60));
  }

  /**
   * An arm's weight pulls hardest at the angle given with its kG: on feedback alone, it sags most
   * there, kG over kP, and not at all a quarter turn on, where it hangs straight.
   */
  @Test
  void anArmsWeightPullsHardestAtTheAngleGiven() {
    Joint.Revolute arm = BARE_ARM.control(blocks(BARE_ARM).withKg(0.4, Degrees.of(-90)));
    MechanismControl feedbackAlone = blocks(arm).withKg(0);
    double hardest = Math.toRadians(-90);

    assertThat(held(arm, blocks(arm), hardest)).isCloseTo(hardest, within(1e-4));
    assertThat(hardest - held(arm, feedbackAlone, hardest))
        .isCloseTo(0.4 / 30, within(0.02 * 0.4 / 30));
    assertThat(held(arm, feedbackAlone, 0)).isCloseTo(0, within(1e-6));
  }

  /**
   * What the robot's settings for a joint would note, the simulator logs too, in the robot's words:
   * a pivot whose tuned kG is a third of what its arm's weight takes notes it, wired to nothing, as
   * it would on the TalonFX it's wired to on a robot.
   */
  @Test
  void itNotesWhatTheRobotsSettingsWould() {
    MechanismControl light = assumed(pivotControl(Optional.of(KG / 3)));
    MechanismWiring onTalon =
        MechanismWiring.builder()
            .canBus(CanBus.SYSTEMCORE)
            .controller(MechanismWiring.Controller.TALON_FX)
            .motors(List.of(MechanismWiring.Motor.builder().id(32).clockwise(true).build()))
            .cancoder(Optional.empty())
            .dataPort(Optional.empty())
            .build();
    List<String> robots =
        PhoenixMechanism.configure(ROBOT.math(), PIVOT, PIVOT_MOTOR, onTalon, light).notes();

    JointIOSim unwired = new JointIOSim(ROBOT.math(), setup(PIVOT, light));
    JointIOSim wired =
        new JointIOSim(
            ROBOT.math(),
            JointSetup.builder().joint(PIVOT).control(light).wiring(Optional.of(onTalon)).build());

    assertThat(robots)
        .singleElement()
        .asString()
        .startsWith("pivot's tuned kG, ")
        .contains("is 0.3333 times the ideal");
    assertThat(unwired.notes()).isEqualTo(robots);
    assertThat(wired.notes()).isEqualTo(robots);
  }

  /**
   * The simulator refuses a joint the robot would refuse at startup, in the robot's words, though
   * its own motor controller could run it: an arm whose profile cruises faster than its motors turn
   * it, on the TalonFX the simulator stands in for, wired to nothing.
   */
  @Test
  void itRefusesAJointTheRobotWouldRefuse() {
    Joint.Revolute fast =
        BARE_ARM.control(
            blocks(BARE_ARM)
                .withProfile(20, 40)
                .withHoming(MechanismControl.Homing.assumeAtReference()));
    RobotDefinition robot = RobotDefinitionBuilder.builder(MASSLESS).joints(List.of(fast)).build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> new JointsSimHarness(robot))
        .withMessage(
            "bare arm's Motion Magic cruise, 3.183 rot/s, is 1.592 times its free speed, 2 rot/s:"
                + " the profile would ask more than its motors turn it at 12 V");
  }
}
