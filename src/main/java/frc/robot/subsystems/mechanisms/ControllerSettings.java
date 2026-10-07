package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.lib.rev.RevMechanism;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * What a driven joint's motor controllers are told, from its setup: a TalonFX's or a TalonFXS's
 * settings from the Phoenix helper ({@link PhoenixMechanism}), a SPARK's from the REV helper
 * ({@link RevMechanism}), with the definition's absolute encoder its wiring reads. Pure, as the
 * helpers are: no device is touched, so they're worked out as the robot's configuration is checked
 * ({@link JointSetup#problems}), in the simulator and the tests as on a robot, and what the robot
 * would refuse at startup is refused there first, with the robot's own message; what it would note,
 * the simulator notes too ({@code JointIOSim}). On a robot, the joint's IO applies these same
 * settings ({@link JointIOTalon}, {@link JointIOSpark}).
 *
 * <p>A joint with no wiring runs in the simulator alone, whose motor controller is a TalonFX's:
 * it's checked on TalonFXs ({@link #notes}).
 */
public final class ControllerSettings {
  private ControllerSettings() {}

  /**
   * What's doubtful in a joint's settings, though allowed, each a sentence: what the robot logs at
   * startup ({@code Joints/<label>/SettingsNotes}), and the simulator with it. Worked out for the
   * controller its wiring names; for a joint with no wiring, for TalonFXs.
   *
   * @throws IllegalArgumentException for what the robot would refuse at startup, saying what and
   *     why as it would
   */
  public static List<String> notes(RobotMath math, JointSetup setup) {
    Joint joint = setup.joint();
    MechanismWiring wiring = setup.wiring().orElse(null);
    if (wiring == null) {
      return talon(math, joint, onTalons(math.drivesOf(joint).get(0)), setup.control()).notes();
    }
    return switch (wiring.controller()) {
      case TALON_FX, TALON_FXS -> talon(math, joint, wiring, setup.control()).notes();
      case SPARK_MAX, SPARK_FLEX -> spark(math, joint, wiring, setup.control()).notes();
    };
  }

  /**
   * A TalonFX's or TalonFXS's settings for a joint, from its wiring and a control: its setup's, or
   * as the pit tunes it. A CANcoder its wiring names is the definition's absolute encoder on it.
   *
   * @throws IllegalArgumentException for a CANcoder the definition doesn't have, and what the
   *     Phoenix helper refuses
   */
  static PhoenixMechanism.Settings talon(
      RobotMath math, Joint joint, MechanismWiring wiring, MechanismControl control) {
    Drive drive = math.drivesOf(joint).get(0);
    if (wiring.cancoder().isEmpty()) {
      return PhoenixMechanism.configure(math, joint, drive, wiring, control);
    }
    Sensor.Encoder encoder = JointSetup.absoluteEncoder(math, joint);
    if (encoder == null) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s wiring names a CANcoder, and its definition has no absolute encoder on it");
    }
    return PhoenixMechanism.configure(math, joint, drive, encoder, wiring, control);
  }

  /**
   * A SPARK MAX's or SPARK Flex's settings for a joint, from its wiring and a control. An encoder
   * on its data port is the definition's absolute encoder on it.
   *
   * @throws IllegalArgumentException for a data-port encoder the definition doesn't have, and what
   *     the REV helper refuses
   */
  static RevMechanism.Settings spark(
      RobotMath math, Joint joint, MechanismWiring wiring, MechanismControl control) {
    Drive drive = math.drivesOf(joint).get(0);
    if (wiring.dataPort().isEmpty()) {
      return RevMechanism.configure(math, joint, drive, wiring, control);
    }
    Sensor.Encoder encoder = JointSetup.absoluteEncoder(math, joint);
    if (encoder == null) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s wiring names a data-port encoder, and its definition has no absolute encoder on"
              + " it");
    }
    return RevMechanism.configure(math, joint, drive, encoder, wiring, control);
  }

  /**
   * The loop a joint closes itself on its absolute encoder on a Systemcore port, for a control: its
   * kP and its feedforward, worked out on the robot as a SPARK's feedforward is, for the controller
   * its wiring names, and a TalonFX for none. Its motor controller's own settings are worked out
   * beside it: they run the joint while the encoder is lost.
   *
   * @throws IllegalArgumentException for what the Phoenix helper refuses
   */
  static EncoderLoop encoderLoop(
      RobotMath math, Joint joint, Optional<MechanismWiring> wiring, MechanismControl control) {
    Drive drive = math.drivesOf(joint).get(0);
    PhoenixMechanism.Settings settings =
        PhoenixMechanism.configureFeedforwardOnRobot(
            math, joint, drive, wiring.orElseGet(() -> onTalons(drive)), control);
    return new EncoderLoop(settings.units(), settings.kP(), settings.feedforward());
  }

  /**
   * What a joint with no wiring is checked as: its motors on TalonFXs, the first leading, each
   * turning counterclockwise for positive, read by their rotors. Their IDs only tell them apart:
   * nothing the Phoenix helper refuses or notes reads an ID, a bus or a direction.
   */
  private static MechanismWiring onTalons(Drive drive) {
    List<MechanismWiring.Motor> motors = new ArrayList<>();
    for (int id = 0; id < ((Actuator.Motors) drive.actuator()).count(); id++) {
      motors.add(MechanismWiring.Motor.builder().id(id).clockwise(false).build());
    }
    return MechanismWiring.builder()
        .canBus(CanBus.SYSTEMCORE)
        .controller(MechanismWiring.Controller.TALON_FX)
        .motors(motors)
        .cancoder(Optional.empty())
        .dataPort(Optional.empty())
        .build();
  }
}
