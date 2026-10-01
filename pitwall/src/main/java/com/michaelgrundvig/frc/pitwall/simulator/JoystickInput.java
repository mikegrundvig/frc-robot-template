package com.michaelgrundvig.frc.pitwall.simulator;

import java.util.List;
import java.util.Set;
import org.jspecify.annotations.Nullable;

/**
 * One joystick's input, as the page sends it from the keyboard or a gamepad: each axis, each
 * button, each POV, by WPILib's index (for a gamepad, {@code org.wpilib.driverstation.Gamepad}'s
 * axes and buttons).
 *
 * @param axes each axis from -1 to 1 (a gamepad's stick is negative pushed up or left)
 * @param buttons whether each button is pressed
 * @param povs each POV's angle in degrees clockwise from up, in steps of 45, or -1 when centered
 */
public record JoystickInput(List<Double> axes, List<Boolean> buttons, List<Integer> povs) {
  /** The most the Driver Station carries for one joystick. */
  public static final int MAX_AXES = 12;

  public static final int MAX_BUTTONS = 64;
  public static final int MAX_POVS = 8;

  /** Nothing pressed, every stick centered. */
  public static final JoystickInput CENTERED = new JoystickInput(List.of(), List.of(), List.of());

  private static final Set<Integer> POV_ANGLES = Set.of(-1, 0, 45, 90, 135, 180, 225, 270, 315);

  public JoystickInput {
    axes = List.copyOf(axes);
    buttons = List.copyOf(buttons);
    povs = List.copyOf(povs);
    String why = whyNot(axes, buttons, povs);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
  }

  /**
   * Why these can't be a joystick's input: too many of them, an axis outside -1 to 1, or a POV
   * angle that isn't one.
   *
   * @return null when they can
   */
  public static @Nullable String whyNot(
      List<Double> axes, List<Boolean> buttons, List<Integer> povs) {
    if (axes.size() > MAX_AXES || buttons.size() > MAX_BUTTONS || povs.size() > MAX_POVS) {
      return "A joystick has at most "
          + MAX_AXES
          + " axes, "
          + MAX_BUTTONS
          + " buttons, and "
          + MAX_POVS
          + " POVs";
    }
    for (double axis : axes) {
      if (!(axis >= -1 && axis <= 1)) {
        return "An axis is from -1 to 1, not " + axis;
      }
    }
    for (int pov : povs) {
      if (!POV_ANGLES.contains(pov)) {
        return "A POV is an angle in steps of 45 degrees from 0 to 315, or -1 centered, not " + pov;
      }
    }
    return null;
  }

  /** Whether every stick is centered and nothing is pressed: nothing for the deadman to stop. */
  public boolean isCentered() {
    return axes.stream().allMatch(axis -> axis == 0)
        && !buttons.contains(true)
        && povs.stream().allMatch(pov -> pov == -1);
  }

  /** The buttons as the Driver Station packs them: button 0 is the lowest bit. */
  public long buttonBits() {
    long bits = 0;
    for (int i = 0; i < buttons.size(); i++) {
      if (buttons.get(i)) {
        bits |= 1L << i;
      }
    }
    return bits;
  }
}
