package com.michaelgrundvig.frc.localization;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * How far localization trusts each of its sources as of an update, and what it bounds: the one
 * place the integrity checks report, each adding what it finds here rather than a channel of its
 * own.
 *
 * @param cameras each camera's trust, by name: those the estimator has weighed a measurement from,
 *     in the order it first did, then the robot's others
 * @param wheels the drivetrain's wheels' trust this loop, from the IMU and the slip check
 * @param protection the bound on the position's error; empty, since nothing computes one yet
 * @param excludedTags the tags left out of every measurement; none yet
 */
public record Integrity(
    Map<String, CameraTrust> cameras,
    WheelTrust wheels,
    Optional<ProtectionLevel> protection,
    Set<Integer> excludedTags) {
  public Integrity {
    cameras = Collections.unmodifiableMap(new LinkedHashMap<>(cameras));
    excludedTags = Set.copyOf(excludedTags);
  }
}
