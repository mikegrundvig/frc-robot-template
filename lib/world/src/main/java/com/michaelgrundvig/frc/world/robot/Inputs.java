package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * The definition's checks of where its switches and port encoders are connected: each input in one
 * pass, from where each is written, its joint's block or the robot's sensors. One sensor on each
 * input, a joint's absolute encoder on a Systemcore port among them; and a switch on a motor
 * controller's limit input only where that controller has one, on the joint whose switch it is. A
 * CANdi's or a CANrange's CAN ID is the devices' to check ({@link CanDevice#sharedIds}), and
 * whether a limit switch's input is the one for its end isn't checked: whether a SPARK's forward
 * input follows its inversion is unconfirmed (open question M22).
 */
final class Inputs {
  private Inputs() {}

  /**
   * What's wrong with where the switches and port encoders are connected, each about the one
   * connected second, or the switch.
   *
   * @param joints the robot's joints, each with its block
   * @param sensors every sensor: each joint's own, then the robot's
   */
  static List<Finding> problems(List<Joint> joints, List<Sensor> sensors) {
    List<Finding> problems = new ArrayList<>();
    Map<SensorWiring.Input, String> taken = new HashMap<>();
    for (Joint joint : joints) {
      Optional<MechanismWiring.PortEncoder> port =
          joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port);
      if (port.isPresent()) {
        take(
            taken,
            SensorWiring.port(port.get().port()),
            joint.label() + " absolute encoder",
            problems);
      }
    }
    Map<String, Joint> byLabel = new HashMap<>();
    joints.forEach(joint -> byLabel.put(joint.label(), joint));
    for (Sensor sensor : sensors) {
      if (!(sensor instanceof Sensor.Switch onSwitch) || onSwitch.wiring().isEmpty()) {
        continue;
      }
      SensorWiring.Input input = onSwitch.wiring().get().input();
      String label = onSwitch.label();
      take(taken, input, label, problems);
      if (!(input instanceof SensorWiring.ControllerLimit limit)) {
        continue;
      }
      if (onSwitch.kind() instanceof Sensor.Switch.Limit own
          && !own.joint().label().equals(limit.joint())) {
        problems.add(
            Finding.refused(
                label,
                label
                    + " is "
                    + possessive(own.joint().label())
                    + " limit switch, on "
                    + possessive(limit.joint())
                    + " controller, which would stop "
                    + limit.joint()
                    + ": wire it to "
                    + possessive(own.joint().label())));
      }
      Joint on = byLabel.get(limit.joint());
      Optional<Driven.Motors> motors =
          on == null
              ? Optional.empty()
              : on.driven().motors().filter(each -> !each.controllers().isEmpty());
      if (motors.isEmpty()) {
        problems.add(
            Finding.refused(
                label,
                label
                    + " is on "
                    + possessive(limit.joint())
                    + " controller's limit input, and "
                    + limit.joint()
                    + " isn't wired: wire "
                    + possessive(limit.joint())
                    + " motors to their controllers, or put "
                    + label
                    + " on a Systemcore port or a CANdi"));
      } else if (motors.get().controllers().get(0).controller()
              == MechanismWiring.Controller.TALON_FX
          && !motors.get().motor().equals(Motor.FALCON_500)
          && !motors.get().motor().equals(Motor.FALCON_500_FOC)) {
        problems.add(
            Finding.refused(
                label,
                label
                    + " is on "
                    + possessive(limit.joint())
                    + " TalonFX's limit input, and a Kraken's TalonFX has none: put it on a"
                    + " CANdi"));
      }
    }
    return problems;
  }

  /** An input taken by what's connected to it, or what's already there named with it. */
  private static void take(
      Map<SensorWiring.Input, String> taken,
      SensorWiring.Input input,
      String label,
      List<Finding> problems) {
    String other = taken.putIfAbsent(input, label);
    if (other != null) {
      problems.add(
          Finding.refused(
              label,
              other
                  + " and "
                  + label
                  + " are on one input, "
                  + SensorWiring.shown(input)
                  + ": move one to an input of its own"));
    }
  }
}
