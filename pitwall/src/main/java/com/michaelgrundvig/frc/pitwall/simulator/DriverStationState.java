package com.michaelgrundvig.frc.pitwall.simulator;

import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;

/**
 * The simulated Driver Station, as the robot program reads it: enabled or not, its mode, its
 * alliance station, whether the field system and the Driver Station are attached, and the match
 * time.
 *
 * @param mode autonomous, teleop, or utility; unknown before a Driver Station sets one
 * @param matchTime the seconds left in the match period, counting down while enabled; -1 for none
 */
public record DriverStationState(
    boolean enabled,
    RobotMode mode,
    AllianceStationID allianceStation,
    boolean fmsAttached,
    boolean dsAttached,
    double matchTime) {
  /** The same, enabled or disabled. Enabling attaches the Driver Station, as one would be. */
  public DriverStationState withEnabled(boolean enabled) {
    return new DriverStationState(
        enabled, mode, allianceStation, fmsAttached, dsAttached || enabled, matchTime);
  }

  /** The same, in another mode. */
  public DriverStationState withMode(RobotMode mode) {
    return new DriverStationState(
        enabled, mode, allianceStation, fmsAttached, dsAttached, matchTime);
  }

  /** The same, at another alliance station. */
  public DriverStationState withAllianceStation(AllianceStationID station) {
    return new DriverStationState(enabled, mode, station, fmsAttached, dsAttached, matchTime);
  }

  /** The same, with the field system attached or not. */
  public DriverStationState withFmsAttached(boolean attached) {
    return new DriverStationState(enabled, mode, allianceStation, attached, dsAttached, matchTime);
  }

  /** The same, with the Driver Station attached or not. Without one, the robot is disabled. */
  public DriverStationState withDsAttached(boolean attached) {
    return new DriverStationState(
        enabled && attached, mode, allianceStation, fmsAttached, attached, matchTime);
  }

  /** The same, with this many seconds left in the match period; -1 for none. */
  public DriverStationState withMatchTime(double seconds) {
    return new DriverStationState(enabled, mode, allianceStation, fmsAttached, dsAttached, seconds);
  }
}
