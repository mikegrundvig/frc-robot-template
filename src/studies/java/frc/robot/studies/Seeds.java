package frc.robot.studies;

/**
 * Seeds made from numbers, so each random draw in a study depends only on what it's for (the run's
 * seed, a pose, a camera), never on the order work happens in. The same numbers give the same seed
 * on every computer.
 *
 * <p>One step of mixing depends on the seed and the number only through their sum, so {@code mix(1,
 * 2)} is {@code mix(2, 1)}, and {@code mix(s, t + 1)} is {@code mix(s + 1, t)}. Numbers mixed in
 * after the first are mixed into a seed already scrambled, so their order matters. Where a run's
 * seed and a count must stay apart (the robustness trials), mix the seed with a constant of its own
 * first.
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
