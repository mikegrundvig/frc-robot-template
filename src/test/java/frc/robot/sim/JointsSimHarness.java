package frc.robot.sim;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.RobotSim;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.mechanisms.AbsoluteEncoderIO;
import frc.robot.subsystems.mechanisms.JointIO;
import frc.robot.subsystems.mechanisms.Joints;
import frc.robot.subsystems.mechanisms.PdhIO;
import frc.robot.subsystems.mechanisms.SensorIO;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.system.RobotController;

/**
 * Runs a robot's driven joints in simulation the way the robot runs them, without the rest of the
 * robot: the joints' physics, then each joint reads its sensors, then the command scheduler, then
 * each joint applies what it was asked. The class using it carries
 * {@code @ExtendWith(RobotSim.class)}, for the stopped clock.
 */
public final class JointsSimHarness {
  public final Scheduler scheduler = Scheduler.createIndependentScheduler();
  public final JointSimulation simulation;
  public final SensorSimulation sensors;
  public final PdhIOSim pdh;
  public final Joints joints;
  private boolean enabled = true;

  /**
   * The definition's joints, each driven as its block says, enabled, each where its definition says
   * it rests.
   */
  public JointsSimHarness(RobotDefinition definition) {
    this(definition, Joints.untuned());
  }

  /** The same, each joint's control as tuned live: read each loop, as on the robot. */
  public JointsSimHarness(
      RobotDefinition definition, Function<Joint, Supplier<MechanismControl>> tuned) {
    simulation = new JointSimulation(definition.math());
    sensors = new SensorSimulation(simulation);
    pdh = new PdhIOSim(definition, () -> simulation, null);
    joints =
        new Joints(
            definition,
            new Joints.Hardware() {
              @Override
              public JointIO joint(Joint joint) {
                return simulation.add(joint);
              }

              @Override
              public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
                return simulation.absoluteEncoder(joint);
              }

              @Override
              public SensorIO sensor(Sensor.Placed sensor) {
                return sensors.add(sensor);
              }

              @Override
              public PdhIO pdh() {
                return pdh;
              }
            },
            tuned,
            scheduler);
  }

  /** Enables the joints, or disables them, from the next loop. */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  /** One robot loop. */
  public void loop() {
    RobotSim.stepClock();
    simulation.step(RobotLoop.PERIOD_SECONDS);
    joints.read(RobotController.getTime(), enabled);
    scheduler.run();
    joints.write();
  }

  /** Robot loops for {@code seconds} of the robot's time. */
  public void run(double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
  }

  /** Schedules a command, then runs loops for {@code seconds}. */
  public void run(Command command, double seconds) {
    scheduler.schedule(command);
    run(seconds);
  }

  /**
   * Robot loops until something holds, at most {@code seconds} of them: how long it took, in
   * seconds, or NaN if it never held.
   */
  public double runUntil(BooleanSupplier done, double seconds) {
    for (int i = 1; i <= RobotLoop.loops(seconds); i++) {
      loop();
      if (done.getAsBoolean()) {
        return i * RobotLoop.PERIOD_SECONDS;
      }
    }
    return Double.NaN;
  }
}
