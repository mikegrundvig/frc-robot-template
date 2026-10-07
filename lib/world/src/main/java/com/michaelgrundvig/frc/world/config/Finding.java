package com.michaelgrundvig.frc.world.config;

/**
 * One thing a check found: what it's about, how much it matters, what reads it, and what to do.
 * Every check across a robot's parts says what it finds this way, in the robot's definition and in
 * the program that drives it, and the program says them in one place: what's refused stops the
 * robot starting ({@link Checks#findings}); a doubt or a note is said at startup, in the log and,
 * for the robot's own, on the Driver Station's console.
 *
 * @param level how much it matters: refused, doubted, or noted
 * @param scope what it matters to: the robot, wherever the program runs, or only the simulator
 * @param subject what it's about, as the robot's logs name it: a joint ("pivot"), a sensor ("pivot
 *     lower limit"), a device ("pivot motor 32"), a configuration ("intaking"), or the robot
 *     itself, by its name
 * @param message what's wrong, doubtful or not known, naming its subject, in the units its values
 *     were given in, and what to do about it
 */
public record Finding(Level level, Scope scope, String subject, String message) {
  /** How much a finding matters. */
  public enum Level {
    /** It can't work: the robot won't start, in the simulator too, and says why. */
    REFUSED,
    /** Allowed, but likely wrong, or costly: said at startup, so someone looks. */
    DOUBT,
    /** Not known, and what not knowing it costs: said at startup, never a guess. */
    NOTE
  }

  /** What reads what a finding is about, and so where it's said. */
  public enum Scope {
    /** The robot's code, its derived values or its checks: said wherever the program runs. */
    ROBOT,
    /** Only the simulator, such as a link's mass: said only when it simulates. */
    SIMULATOR
  }

  public Finding {
    Checks.on(Finding.class, subject)
        .field("subject", subject, Rules.notBlank())
        .field("message", message, Rules.notBlank())
        .orThrow();
  }

  /** What can't work, about the robot: it won't start. */
  public static Finding refused(String subject, String message) {
    return new Finding(Level.REFUSED, Scope.ROBOT, subject, message);
  }

  /** What's doubtful though allowed, about the robot. */
  public static Finding doubt(String subject, String message) {
    return new Finding(Level.DOUBT, Scope.ROBOT, subject, message);
  }

  /** What isn't known, and what that costs whatever reads it. */
  public static Finding note(Scope scope, String subject, String message) {
    return new Finding(Level.NOTE, scope, subject, message);
  }

  /** Whether it stops the robot starting. */
  public boolean refused() {
    return level == Level.REFUSED;
  }

  /** Its message: what a log line or a refusal says. */
  @Override
  public String toString() {
    return message;
  }
}
