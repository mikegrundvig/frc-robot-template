package com.michaelgrundvig.frc.world.field;

import org.wpilib.math.geometry.Translation2d;

/**
 * A zone as a trigger: on while the robot is inside, or will be within a lookahead at its current
 * velocity, and off again only once it is out by a margin. Strong teams act before arriving (a
 * shooter lowered 0.4 s before a low bar), and the margin keeps a robot on the edge from flickering
 * the trigger on and off.
 *
 * <p>Call {@link #update} once per loop.
 */
public final class ZoneTrigger {
  private final Zone zone;
  private final double lookaheadSeconds;
  private final double exitMarginMeters;
  private boolean active;

  /**
   * @param lookaheadSeconds how far ahead to look along the robot's path
   * @param exitMarginMeters how far out of the zone the robot must be to turn the trigger off
   */
  public ZoneTrigger(Zone zone, double lookaheadSeconds, double exitMarginMeters) {
    if (!(lookaheadSeconds >= 0) || !(exitMarginMeters >= 0)) {
      throw new IllegalArgumentException("the lookahead and the margin must be zero or more");
    }
    this.zone = zone;
    this.lookaheadSeconds = lookaheadSeconds;
    this.exitMarginMeters = exitMarginMeters;
  }

  /**
   * Updates the trigger from where the robot is and how it is moving.
   *
   * @param fieldVelocity field-relative, meters per second
   * @return whether the trigger is on
   */
  public boolean update(
      FieldModel field, Alliance alliance, Translation2d position, Translation2d fieldVelocity) {
    boolean arriving =
        zone.reachedWithin(field, alliance, position, fieldVelocity, lookaheadSeconds);
    active = arriving || (active && zone.distance(field, alliance, position) <= exitMarginMeters);
    return active;
  }

  /** Whether the trigger is on, as of the last update. */
  public boolean isActive() {
    return active;
  }

  public Zone zone() {
    return zone;
  }
}
