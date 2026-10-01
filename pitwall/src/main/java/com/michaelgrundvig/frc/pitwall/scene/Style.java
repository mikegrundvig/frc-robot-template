package com.michaelgrundvig.frc.pitwall.scene;

import java.util.Locale;

/**
 * How a shape is drawn: one of the page's colors, by what it means, and solid or dashed. The page
 * takes each color from its own theme, light or dark, so a scene never names a color; and since
 * color alone never carries meaning on the page, the line itself does too: a dashed line is
 * something not to trust as drawn (a rejected measurement, a guess, the simulator's truth beside
 * the estimate).
 *
 * @param token what the color says: one of the page's color tokens
 * @param stroke solid or dashed
 */
public record Style(Token token, Stroke stroke) {
  /** What a color says. */
  public enum Token {
    /** Good: accepted, passing. */
    OK,
    /** Wrong: rejected, failing. */
    BAD,
    /** Background: the field, what frames the rest. */
    MUTED,
    /** The subject: the robot as the code believes it to be. */
    ACCENT,
    /** The simulator's truth, where the robot really is. */
    TRUTH
  }

  /** How its lines are drawn. */
  public enum Stroke {
    SOLID,
    DASHED
  }

  public static final Style OK = new Style(Token.OK, Stroke.SOLID);
  public static final Style BAD = new Style(Token.BAD, Stroke.SOLID);
  public static final Style MUTED = new Style(Token.MUTED, Stroke.SOLID);
  public static final Style ACCENT = new Style(Token.ACCENT, Stroke.SOLID);
  public static final Style TRUTH = new Style(Token.TRUTH, Stroke.SOLID);

  /** The same color, dashed. */
  public Style dashed() {
    return new Style(token, Stroke.DASHED);
  }

  /** The same color, solid. */
  public Style solid() {
    return new Style(token, Stroke.SOLID);
  }

  /**
   * As the page reads it: the token's name, then {@code dashed} if it is ({@code "bad dashed"}).
   */
  String wire() {
    String name = token.name().toLowerCase(Locale.ROOT);
    return stroke == Stroke.DASHED ? name + " dashed" : name;
  }
}
