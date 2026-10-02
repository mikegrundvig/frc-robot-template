package com.michaelgrundvig.frc.simulation;

/**
 * Where each of the simulated world's random streams starts, from the world's one seed ({@code
 * SensorErrors.seed}), so a run repeats exactly and each stream draws its own numbers. The cameras'
 * come from the robot program's camera simulation, beside PhotonVision's ({@code
 * VisionSim.cameraSeed}).
 *
 * <p>These are offsets from the seed, kept as they were so recorded runs don't move. They don't
 * keep every stream apart: seed 1's gyro draws what its first camera does, and each seed's gyro
 * what the seed before it gave its gyro's accelerometer (the truth suite keeps this as a known gap,
 * in the robot program's {@code SeedsTruthSimTest}, beside the cameras' streams). Deriving each
 * stream from the seed and its own name fixes that, and moves every recording.
 */
public final class Seeds {
  /** The backup IMU's streams start this far from the gyro's: no small seed's gyro shares one. */
  static final long BACKUP_OFFSET = 1L << 32;

  private Seeds() {}

  /** The gyro's noise. */
  public static long gyro(long seed) {
    return seed;
  }

  /** The gyro's accelerometer's noise, on its forward and left axes. */
  public static long gyroAccelerometer(long seed) {
    return seed + 1;
  }

  /** The backup IMU's noise. */
  public static long backupGyro(long seed) {
    return seed + BACKUP_OFFSET;
  }

  /** The backup IMU's accelerometer's noise, on its forward and left axes. */
  public static long backupAccelerometer(long seed) {
    return seed + BACKUP_OFFSET + 1;
  }

  /**
   * An accelerometer's noise on its up axis, a stream apart from its other two, so they draw as
   * they would without it.
   *
   * @param accelerometer the seed of its other two axes
   */
  public static long upAxis(long accelerometer) {
    return ~accelerometer;
  }
}
