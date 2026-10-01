package frc.robot.studies;

/**
 * Seeds made from numbers, so each random draw in a study depends only on what it's for (the run's
 * seed, a pose, a camera), never on the order work happens in. The same numbers give the same seed
 * on every computer.
 */
final class Seeds {
  private Seeds() {}

  /** A seed from a seed and numbers. */
  static long mix(long seed, double... values) {
    long mixed = seed;
    for (double value : values) {
      mixed = mix(mixed, Double.doubleToLongBits(value));
    }
    return mixed;
  }

  /** A seed from a seed and a number: SplitMix64's mixing, so nearby numbers seed far apart. */
  static long mix(long seed, long value) {
    long h = seed + value + 0x9E3779B97F4A7C15L;
    h = (h ^ (h >>> 30)) * 0xBF58476D1CE4E5B9L;
    h = (h ^ (h >>> 27)) * 0x94D049BB133111EBL;
    return h ^ (h >>> 31);
  }
}
