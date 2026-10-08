package com.michaelgrundvig.frc.pitwall;

import java.util.List;

/**
 * What became of a request from the page, once the robot loop handled it.
 *
 * @param result whether it was applied, refused, or locked out
 * @param detail why it wasn't applied, one reason a line; empty when it was
 * @param reasons for a change to tunables that was refused, each reason with the values it
 *     concerns, in {@code detail}'s order; empty otherwise
 */
public record Outcome(Result result, String detail, List<Reason> reasons) {
  /** Whether a request was applied, and if not, why not. */
  public enum Result {
    /** Applied, all of it. */
    APPLIED,
    /** Refused by the configuration's checks or by the action itself; nothing changed. */
    REFUSED,
    /**
     * Locked out: tuning is off, the robot isn't enabled in utility mode (for actions and
     * experiments), or the field system is connected; or there is nothing to act on, such as a
     * cancel for an experiment that isn't running. Nothing changed.
     */
    LOCKED
  }

  /**
   * Which check refused a change. They run in this order, and a change is refused at the first that
   * finds anything wrong, so a later one may not have run at all.
   */
  public enum Layer {
    /** The value isn't one the page can change, or isn't of its type. */
    VALUE,
    /** The value is outside its tuning limits. */
    LIMIT,
    /** A record of its configuration refused it, when rebuilt with the change. */
    RECORD,
    /** The tunable's own setter threw. */
    TUNABLE
  }

  /**
   * One reason a change to tunables was refused, tied to the values it concerns, so the page can
   * show it beside each of them.
   *
   * @param layer which check refused it
   * @param by who refused it: the record whose check it was ({@code DrivetrainControl.Gains});
   *     empty for a value's own limits and type
   * @param paths the tunables it concerns, each to show it beside: the one a value's own check (its
   *     type, its limits) or a record's check names; for a record's check that names none (a check
   *     of the record as a whole), every value of the change in that record; for a tunable's own
   *     setter, the tunables it was setting
   * @param config the configuration it concerns; empty for a tunable of no configuration
   * @param group for a record's check that names no value, where that record is in its
   *     configuration ({@code travel}; empty for the root); empty otherwise
   * @param text why, as the check said it
   * @param line the reason's line in {@link #detail}
   */
  public record Reason(
      Layer layer,
      String by,
      List<String> paths,
      String config,
      String group,
      String text,
      String line) {
    public Reason {
      paths = List.copyOf(paths);
    }

    /** A reason that concerns one tunable, of no configuration's record. */
    static Reason of(Layer layer, String path, String config, String text, String line) {
      return new Reason(layer, "", List.of(path), config, "", text, line);
    }
  }

  public Outcome {
    reasons = List.copyOf(reasons);
  }

  /** An outcome with no reasons tied to values: a lock, or a refusal of something else. */
  public Outcome(Result result, String detail) {
    this(result, detail, List.of());
  }

  /** A request applied, all of it. */
  public static Outcome applied() {
    return new Outcome(Result.APPLIED, "");
  }

  /** A change refused for these reasons, a line of {@link #detail} for each. */
  static Outcome refused(List<Reason> reasons) {
    return new Outcome(
        Result.REFUSED, String.join("\n", reasons.stream().map(Reason::line).toList()), reasons);
  }
}
