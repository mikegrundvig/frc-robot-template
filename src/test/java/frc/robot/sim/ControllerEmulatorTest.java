package frc.robot.sim;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.gearing;
import static frc.robot.subsystems.mechanisms.JointFixtures.unmotored;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.RobotSim;
import frc.robot.lib.wiring.CanPorts;
import frc.robot.lib.wiring.ControllerPins;
import frc.robot.subsystems.mechanisms.JointIO;
import frc.robot.subsystems.mechanisms.JointIOSpark;
import frc.robot.subsystems.mechanisms.Move;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.simulation.SimDeviceSim;

/**
 * The simulator's motor controllers, each evaluating the joint's plan where its controller does: a
 * TalonFX's slot, all of it on the device every millisecond; a SPARK's kP on the device, with the
 * feedforward the robot works out as it asks, held between requests. The fixtures' pivot, with
 * friction, damping and an integral tuned.
 */
@ExtendWith(RobotSim.class)
class ControllerEmulatorTest {
  private static final Joint.Revolute TUNED =
      PIVOT.retuned(control -> control.withKs(0.2).withKi(2).withKd(1.5));

  private static final ControllerPlan PLAN =
      ControllerPlan.of(ROBOT.math(), TUNED, TUNED.driven().control().orElseThrow());

  /**
   * A TalonFX's slot runs the whole law each millisecond, at where it reads the joint: the feedback
   * on the error and its integral, the damping toward the speed asked from the speed it reads, the
   * plan's feedforward at its position, and the program's own volts.
   */
  @Test
  void aTalonFxsSlotRunsTheWholeLawWhereItReadsTheJoint() {
    TalonFxEmulator talon = new TalonFxEmulator(PLAN);

    talon.hold(1.0, 0.5, 0.3, 0.9, 0.1, true);
    double first = talon.holding(0.95, 0.4, 0.001);
    double second = talon.holding(0.95, 0.4, 0.001);
    talon.run(2.0, 0.9);

    double feedback = PLAN.kP() * 0.05 + PLAN.kD() * (0.5 - 0.4);
    assertThat(first)
        .isCloseTo(
            feedback + PLAN.kI() * 0.05 * 0.001 + PLAN.feedforward(0.95, 0.5, 0) + 0.3,
            within(1e-12));
    assertThat(second - first).as("its integral grows").isCloseTo(2 * 0.05 * 0.001, within(1e-12));
    assertThat(talon.running(1.1, 1.5))
        .isCloseTo(PLAN.kP() * 0.5 + PLAN.feedforward(1.1, 2.0, 0), within(1e-12));
  }

  /**
   * A SPARK runs its kP on the device; the rest the robot worked out as it asked, from where it had
   * last read the joint, and the SPARK holds it whatever the joint does until it's asked again.
   */
  @Test
  void aSparkRunsItsKpAndHoldsWhatTheRobotWorkedOut() {
    SparkEmulator spark = new SparkEmulator(PLAN);

    spark.hold(1.0, 0.5, 0.3, 0.9, 0.1, true);
    double held = 0.3 + PLAN.feedforward(1.0, 0.5, 0) + PLAN.kD() * (0.5 - 0.1);
    double near = spark.holding(0.95, 0.4, 0.001);
    double there = spark.holding(1.0, 7, 0.001);
    spark.run(2.0, 0.9);

    assertThat(near).isCloseTo(PLAN.kP() * 0.05 + held, within(1e-12));
    assertThat(there).as("no damping on the device").isCloseTo(held, within(1e-12));
    assertThat(spark.running(1.1, 1.5))
        .as("gravity where it was last read")
        .isCloseTo(PLAN.kP() * 0.5 + PLAN.feedforward(0.9, 2.0, 0), within(1e-12));
  }

  /**
   * A joint wired to a SPARK is simulated as one, and asked what the robot's own SPARK is: held at
   * where it rests, the volts the simulator puts across its motor are the feedforward the robot
   * sends REV's simulated SPARK for the same request.
   */
  @Test
  void theSimulatorsSparkIsAskedWhatTheRobotsIs() {
    MechanismWiring.Device wired = SPARK_MAX.can(62);
    // A SPARK takes no kI yet.
    Joint.Revolute pivot =
        unmotored(PIVOT)
            .motor(Motor.NEO, gearing(25), wired)
            .retuned(control -> control.withKs(0.2).withKd(1.5));
    RobotDefinition robot = ROBOT.with(pivot);
    JointIOSpark onRobot = new JointIOSpark(robot.math(), pivot, ControllerPins.NONE);
    JointIO simulated = new JointSimulation(robot.math()).add(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    double rest = PIVOT.reference();

    onRobot.updateInputs(inputs);
    onRobot.setPosition(rest, 0.4, 0.25);
    simulated.updateInputs(inputs);
    simulated.setPosition(rest, 0.4, 0.25);
    ((JointIOSim) simulated).step(JointIOSim.DEVICE_LOOP_SECONDS, 0);
    simulated.updateInputs(inputs);

    SimDeviceSim spark =
        new SimDeviceSim(
            String.format(
                Locale.ROOT,
                "SPARK MAX [%d,%d]",
                CanPorts.of(((CanBus.Port) wired.bus()).port()).value,
                wired.id()));
    assertThat(simulated).isInstanceOf(JointIOSim.class);
    // A SPARK holds its feedforward as a float.
    assertThat(inputs.appliedVolts)
        .isCloseTo(spark.getDouble("Arbitrary Feedforward").get(), within(1e-6))
        .isPositive();
  }

  /** On a SPARK in the simulator, the pivot goes out as it does on a TalonFX. */
  @Test
  void aJointOnASparkArrivesInTheSimulator() {
    JointsSimHarness sim =
        new JointsSimHarness(
            ROBOT.with(unmotored(PIVOT).motor(Motor.NEO, gearing(25), SPARK_MAX.can(32))));
    Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);

    sim.scheduler.schedule(out);
    sim.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(out.arrived()).isTrue();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(130, within(2.0));
  }
}
