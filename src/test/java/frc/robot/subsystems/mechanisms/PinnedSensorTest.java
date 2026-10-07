package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.RobotSim;
import frc.robot.behavior.Gate;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;

/**
 * A switch on a joint's motor controller's limit input is read by that joint's IO, with the rest of
 * the joint, and gated on like any sensor: its controller lost, so is it.
 */
@ExtendWith(RobotSim.class)
class PinnedSensorTest {
  private static final Sensor.Switch PIVOT_DOWN =
      Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, 0);

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(PIVOT_DOWN)).build();

  /** The pivot's controller, as a test leaves it: answering, its reverse input as set. */
  private static final class Controller implements JointIO {
    boolean connected = true;
    boolean reverse;

    @Override
    public void updateInputs(JointIOInputs inputs) {
      inputs.connected = connected;
      inputs.reverseLimit = reverse;
    }
  }

  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final Controller pivot = new Controller();
  private final Joints joints =
      new Joints(
          ROBOT,
          new Joints.Hardware() {
            @Override
            public JointIO joint(Joint joint) {
              return joint.equals(PIVOT) ? pivot : new JointIO() {};
            }

            @Override
            public SensorIO sensor(Sensor.Placed sensor) {
              return SensorIO.onPin("pivot", SensorWiring.Direction.REVERSE);
            }
          },
          Joints.untuned(),
          scheduler);

  @Test
  void aSwitchOnAJointsPinIsReadThroughItsController() {
    Gate down = joints.sensors().gate(PIVOT_DOWN.tripped());

    pivot.reverse = true;
    joints.read(0, false);
    scheduler.run();
    boolean tripped = down.getAsBoolean();
    pivot.connected = false;
    joints.read(20_000_000, false);
    scheduler.run();

    assertThat(tripped).isTrue();
    assertThat(down.getAsBoolean()).as("its controller silent").isFalse();
    assertThat(down.blocking()).containsExactly("pivot down lost");
  }

  @Test
  void aSwitchOnAJointTheTemplateDoesntDriveIsRefused() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                new Joints(
                    ROBOT,
                    new Joints.Hardware() {
                      @Override
                      public JointIO joint(Joint joint) {
                        return new JointIO() {};
                      }

                      @Override
                      public SensorIO sensor(Sensor.Placed sensor) {
                        return SensorIO.onPin("winch", SensorWiring.Direction.REVERSE);
                      }
                    },
                    Joints.untuned(),
                    scheduler))
        .withMessage("a switch is on winch's limit input, and the template doesn't drive it");
  }
}
