package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotSim;
import frc.robot.behavior.Outcome;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.JointSimulation;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;
import org.wpilib.system.RobotController;

/**
 * A joint whose motor controllers stop answering: its moves are refused, at the start or mid-way,
 * with the reason. The simulator's joints always answer, so each here can be silenced.
 */
@ExtendWith(RobotSim.class)
class JointFaultsSimTest {
  /** A simulated joint that can be silenced: it reads as a motor controller that isn't there. */
  private static final class Silenceable implements JointIO {
    private final JointIO joint;
    boolean silent;
    String unconfigured = "";

    Silenceable(JointIO joint) {
      this.joint = joint;
    }

    @Override
    public void updateInputs(JointIOInputs inputs) {
      joint.updateInputs(inputs);
      inputs.connected = !silent;
      inputs.unconfigured = unconfigured;
    }

    @Override
    public void setPosition(double position, double velocity, double feedforwardVolts) {
      joint.setPosition(position, velocity, feedforwardVolts);
    }

    @Override
    public void setVelocity(double velocity) {
      joint.setVelocity(velocity);
    }

    @Override
    public void setVoltage(double volts) {
      joint.setVoltage(volts);
    }

    @Override
    public void setDutyCycle(double share) {
      joint.setDutyCycle(share);
    }

    @Override
    public void stop() {
      joint.stop();
    }
  }

  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final JointSimulation simulation = new JointSimulation(ROBOT.math());
  private Silenceable pivotIo = new Silenceable(new JointIO() {});
  private final Joints joints =
      new Joints(
          ROBOT,
          SETUPS,
          setup -> {
            Silenceable io = new Silenceable(simulation.add(setup));
            if (setup.joint().equals(PIVOT)) {
              pivotIo = io;
            }
            return io;
          },
          scheduler);
  private final JointMechanism pivot = joints.of(PIVOT);

  private void loop() {
    RobotSim.stepClock();
    simulation.step(RobotLoop.PERIOD_SECONDS);
    joints.read(RobotController.getTime(), true);
    scheduler.run();
    joints.write();
  }

  private void loopUntil(BooleanSupplier done) {
    for (int i = 0; i < RobotLoop.loops(3) && !done.getAsBoolean(); i++) {
      loop();
    }
  }

  @Test
  void aJointThatIsntAnsweringRefusesItsMoves() {
    pivotIo.silent = true;
    loop();
    Move out = pivot.goTo("out");

    scheduler.schedule(out);
    loopUntil(() -> out.outcome().isPresent());

    assertThat(out.outcome()).contains(Outcome.REFUSED);
    assertThat(out.reason())
        .isEqualTo("pivot → out: REFUSED: pivot isn't answering: its motor controllers are silent");
  }

  /**
   * A motor controller that didn't take its settings (the survey found configs built and never
   * applied) has no current limits or ratio: the joint is faulted, and nothing drives it.
   */
  @Test
  void aJointWhoseMotorDidntTakeItsSettingsIsFaulted() {
    pivotIo.unconfigured = "motor 11";
    loop();
    Move out = pivot.goTo("out");

    scheduler.schedule(out);
    loopUntil(() -> out.outcome().isPresent());

    assertThat(pivot.state()).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(out.outcome()).contains(Outcome.REFUSED);
    assertThat(out.reason())
        .isEqualTo(
            "pivot → out: REFUSED: pivot's motor 11 didn't take its settings: check its CAN ID and"
                + " wiring, then restart the robot");
  }

  @Test
  void aJointThatFallsSilentMidWayEndsItsMove() {
    Move out = pivot.goTo("out");
    scheduler.schedule(out);
    for (int i = 0; i < 10; i++) {
      loop();
    }

    pivotIo.silent = true;
    loopUntil(() -> out.outcome().isPresent());

    assertThat(out.outcome()).contains(Outcome.REFUSED);
    assertThat(out.reason())
        .isEqualTo(
            "pivot → out: REFUSED: can't go on: pivot isn't answering: its motor controllers are"
                + " silent");
  }

  /**
   * On a robot without its wiring, a joint has no hardware, and nothing answers. (In a replay its
   * readings, whether it answered among them, come from the log.)
   */
  @Test
  void aJointWithNoHardwareRefusesItsMoves() {
    Scheduler alone = Scheduler.createIndependentScheduler();
    Joints none = new Joints(ROBOT, SETUPS, setup -> new JointIO() {}, alone);
    Move out = none.of(PIVOT).goTo("out");

    alone.schedule(out);
    for (int i = 0; i < 3; i++) {
      RobotSim.stepClock();
      none.read(RobotController.getTime(), true);
      alone.run();
      none.write();
    }

    assertThat(out.outcome()).contains(Outcome.REFUSED);
  }
}
