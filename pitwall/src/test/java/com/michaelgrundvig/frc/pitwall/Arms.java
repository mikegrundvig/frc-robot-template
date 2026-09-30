package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * Two arms that swing in one plane and can collide, but only if both reach into each other's space:
 * the worked example of a rule across configurations. An intake, low at the front, swings from
 * forward (toward the hood) up to past vertical; a hood above and ahead of it swings from down and
 * back (toward the intake) up to vertical. Each is its own mechanism with its own configuration.
 *
 * <p>The geometry is kept simple: each arm sweeps a sector of a circle, and two arms are clear when
 * the sectors they sweep stay a clearance apart, found by sampling each sector's outline every
 * degree and every centimeter.
 */
final class Arms {
  private Arms() {}

  /**
   * An arm's swing, in degrees counterclockwise from the robot's forward direction.
   *
   * @param from where it starts
   * @param to where it ends, counterclockwise from {@code from}
   */
  record Swing(double from, double to) {
    Swing {
      if (!(from < to)) {
        throw new IllegalArgumentException("Swing.to: must be past from (" + from + ")");
      }
    }
  }

  /**
   * One arm.
   *
   * @param name what it is, for messages
   * @param pivotX its pivot, forward of the robot's center, in meters
   * @param pivotY its pivot, above the floor, in meters
   * @param length from its pivot to its tip, in meters: not tunable, it's the arm
   * @param swing the range it swings through: tunable
   * @param kP its position gain: tunable, and no rule reads it
   */
  record Arm(String name, double pivotX, double pivotY, double length, Swing swing, double kP) {}

  /** The intake: 0.5 m long, swinging from 45° up to 120°. */
  static Arm intake() {
    return new Arm("intake", 0, 0, 0.5, new Swing(45, 120), 4);
  }

  /** The hood: 0.4 m long, 0.6 m ahead of the intake's pivot and 0.4 m above it, 220° to 270°. */
  static Arm hood() {
    return new Arm("hood", 0.6, 0.4, 0.4, new Swing(220, 270), 2);
  }

  /** The intake and the hood as tunable configurations: their swings, and their gains. */
  static LiveConfig<Arm> live(Arm arm) {
    return LiveConfig.of(arm.name(), arm)
        .number("swing.from", Limits.between(-180, 360))
        .number("swing.to", Limits.between(-180, 360))
        .number("kP", Limits.between(0, 20));
  }

  /**
   * Why two arms can collide, or null when they're clear: the sectors they sweep come within the
   * clearance, in meters.
   */
  static @Nullable String clear(Arm a, Arm b, double clearance) {
    double gap = gap(a, b);
    if (gap >= clearance) {
      return null;
    }
    return String.format(
        Locale.ROOT,
        "the %s (%.0f° to %.0f°) and the %s (%.0f° to %.0f°) would come within %.1f cm;"
            + " they need %.1f cm",
        a.name(),
        a.swing().from(),
        a.swing().to(),
        b.name(),
        b.swing().from(),
        b.swing().to(),
        gap * 100,
        clearance * 100);
  }

  /** How close the sectors two arms sweep come, in meters. */
  static double gap(Arm a, Arm b) {
    List<Translation2d> first = outline(a);
    List<Translation2d> second = outline(b);
    double closest = Double.POSITIVE_INFINITY;
    for (Translation2d p : first) {
      for (Translation2d q : second) {
        closest = Math.min(closest, p.getDistance(q));
      }
    }
    return closest;
  }

  /** Points around the sector an arm sweeps: its arc every degree, its two ends every cm. */
  private static List<Translation2d> outline(Arm arm) {
    Translation2d pivot = new Translation2d(arm.pivotX(), arm.pivotY());
    double from = arm.swing().from();
    double to = arm.swing().to();
    List<Translation2d> points = new ArrayList<>();
    int degrees = (int) Math.ceil(to - from);
    for (int i = 0; i <= degrees; i++) {
      points.add(tip(pivot, arm.length(), from + (to - from) * i / degrees));
    }
    int centimeters = (int) Math.ceil(arm.length() * 100);
    for (double angle : new double[] {from, to}) {
      for (int j = 0; j <= centimeters; j++) {
        points.add(tip(pivot, arm.length() * j / centimeters, angle));
      }
    }
    return points;
  }

  private static Translation2d tip(Translation2d pivot, double length, double degrees) {
    double radians = Math.toRadians(degrees);
    return pivot.plus(new Translation2d(length * Math.cos(radians), length * Math.sin(radians)));
  }
}
