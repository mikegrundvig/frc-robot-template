package com.michaelgrundvig.frc.pitwall.docs;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.Map;
import org.junit.jupiter.api.Test;

/** Doc comments as the compiler gives them: each line without its leading star or slashes. */
class DocCommentTest {
  @Test
  void theSummaryIsTheFirstSentence() {
    assertThat(traditional(" Gains. Tuned live.\n").summary()).isEqualTo("Gains.");
    assertThat(traditional(" Gains, version 1.5 of them\n\n<p>More.").summary())
        .as("a period inside a number doesn't end it, and a paragraph does")
        .isEqualTo("Gains, version 1.5 of them");
    assertThat(traditional(" Built by {@code Gains.builder(). steerKp(16)}. Tuned live.").summary())
        .as("nor does a period in code")
        .isEqualTo("Built by `Gains.builder(). steerKp(16)`.");
    assertThat(traditional(" @param x a value\n").summary()).isEmpty();
    assertThat(traditional(" A ` mark. More.").summary())
        .as("a stray backtick is just a character")
        .isEqualTo("A ` mark.");
  }

  @Test
  void codeKeepsItsBracesAndItsAtSigns() {
    DocComment doc =
        traditional(
            " Built as {@code new double[] {1, 2}} or {@code\n @Config record X()}.\n"
                + " @param x the {@code {}} value\n");

    assertThat(doc.text()).isEqualTo("Built as `new double[] {1, 2}` or `@Config record X()`.");
    assertThat(doc.params()).containsExactly(Map.entry("x", "the `{}` value"));
  }

  @Test
  void aLinkShowsItsLabelOrWhatItNames() {
    assertThat(plain("{@link com.michaelgrundvig.frc.world.robot.DrivetrainControl.Gains}"))
        .isEqualTo("DrivetrainControl.Gains");
    assertThat(plain("{@link Limits#between(double, double)}"))
        .isEqualTo("between(double, double)");
    assertThat(plain("{@linkplain Limits#between(double, double) its limits}"))
        .isEqualTo("its limits");
    assertThat(plain("{@link #NONE}")).isEqualTo("NONE");
    assertThat(plain("at most {@value Limits#MAX_CURRENT} amps"))
        .isEqualTo("at most MAX_CURRENT amps");
  }

  @Test
  void otherMarkupIsDropped() {
    assertThat(plain("a <a href=\"https://wpilib.org\">WPILib</a> link<br>next"))
        .isEqualTo("a WPILib link next");
    assertThat(plain("x &lt; y &#38; y &#x3E; z, &unknown; a < b, <!-- hidden -->c"))
        .isEqualTo("x < y & y > z, &unknown; a < b, c");
    assertThat(plain("{@literal <p>}{@inheritDoc}")).isEqualTo("<p>");
  }

  @Test
  void unitsWrittenAsEntitiesAreTheirSymbols() {
    assertThat(plain("5&deg; &plusmn; 0.5&deg;, 2 &times; 3 &micro;s, 4.7 k&Omega;"))
        .isEqualTo("5° ± 0.5°, 2 × 3 µs, 4.7 kΩ");
    assertThat(plain("0 &le; x &ge; 1 &rarr; y &mdash; z &ndash; w"))
        .isEqualTo("0 ≤ x ≥ 1 → y — z – w");
  }

  @Test
  void controlCharactersAreDroppedAndWhitespaceIsSpace() {
    assertThat(plain("a&#9;b&#10;c&#13;d")).isEqualTo("a b c d");
    assertThat(plain("a&#7;b&#x1F;c&#127;d&#x80;e&#159;f")).isEqualTo("abcdef");
    assertThat(plain("a\u0007b\u007Fc\u0085d")).as("written as they are").isEqualTo("abcd");
    assertThat(plain("{@code a\u0007b}")).isEqualTo("`ab`");
    assertThat(plain("&#xD800; &#99999999999; &#x110000; &#-5; &#xZZ;"))
        .as("numbers that aren't characters stay as written")
        .isEqualTo("&#xD800; &#99999999999; &#x110000; &#-5; &#xZZ;");
  }

  @Test
  void codeHoldingABacktickGetsMoreOfThem() {
    assertThat(plain("{@code a`b} and {@code `c}")).isEqualTo("``a`b`` and `` `c ``");
  }

  @Test
  void listsAndParagraphsKeepTheirShape() {
    assertThat(plain("Kinds:<ol><li>one<li>two</li></ol>Then <P>more</P>"))
        .isEqualTo("Kinds:\n- one\n- two\n\nThen\n\nmore");
  }

  @Test
  void markdownCommentsReadTheSameWay() {
    DocComment doc =
        DocComment.parse(
            """
            Gains for [SwerveConfig], set with `steerKp(16)`: see [the docs][Docs#gains].

            Two kinds, in [order](https://docs.wpilib.org):
            - drive, `List<Double>` of them
            * steer, {@code Map<K, V>}
            Then [] and [unclosed.

            @param steerKp volts per [radian][Units#Radians] of error
            """,
            true);

    assertThat(doc.summary())
        .isEqualTo("Gains for SwerveConfig, set with `steerKp(16)`: see the docs.");
    assertThat(doc.text())
        .isEqualTo(
            "Gains for SwerveConfig, set with `steerKp(16)`: see the docs.\n\n"
                + "Two kinds, in order:\n"
                + "- drive, `List<Double>` of them\n"
                + "- steer, `Map<K, V>` Then [] and [unclosed.");
    assertThat(doc.params()).containsExactly(Map.entry("steerKp", "volts per radian of error"));
  }

  /** Brackets are a link only when they name something: otherwise they're text. */
  @Test
  void markdownBracketsThatArentLinksStay() {
    assertThat(markdown("in [0, 1], values[0], matrix[0][1], [], [a b], and [unclosed"))
        .isEqualTo("in [0, 1], values[0], matrix[0][1], [], [a b], and [unclosed");
    assertThat(markdown("[SwerveConfig], [#steerKp], [Limits#between(double, double)]"))
        .isEqualTo("SwerveConfig, steerKp, between(double, double)");
    assertThat(markdown("[the gains][SwerveConfig.Gains], [a page](https://wpilib.org), [x][1]"))
        .isEqualTo("the gains, a page, x[1]");
    assertThat(markdown("[`steerKp`][#steerKp]")).isEqualTo("`steerKp`");
  }

  /** Code holds anything, braces and at signs included, but only within its paragraph. */
  @Test
  void markdownCodeIsCodeWithinItsParagraph() {
    assertThat(markdown("`{` opens, `}` closes, and `a{b` is text: {@code x}"))
        .isEqualTo("`{` opens, `}` closes, and `a{b` is text: `x`");
    assertThat(markdown("one `tick\n\nthen` another")).isEqualTo("one `tick\n\nthen` another");
    assertThat(markdown("``a `b` c`` and ```d``")).isEqualTo("``a `b` c`` and ```d``");

    DocComment doc =
        DocComment.parse(
            """
            Built so:
            ```java
            @ConfigRecord
            record X(double kP) {}
            ```
            and `
            @Override` too.
            \\@not a tag
            @param kP volts
            """,
            true);

    assertThat(doc.text())
        .isEqualTo(
            "Built so:\n\n`@ConfigRecord record X(double kP) {}`\n\nand `@Override` too. @not a tag");
    assertThat(doc.params()).containsExactly(Map.entry("kP", "volts"));
  }

  @Test
  void markdownEscapesAndNumberedLists() {
    assertThat(markdown("\\[x\\] \\`y\\` \\& \\a")).isEqualTo("[x] `y` & \\a");
    assertThat(markdown("Steps:\n1. home\n2. zero\n\n3) run"))
        .isEqualTo("Steps:\n- home\n- zero\n- run");
    assertThat(markdown("Bought in\n2020. Then sold")).isEqualTo("Bought in 2020. Then sold");
    assertThat(markdown("+ one\n  more of it\n2. two")).isEqualTo("- one more of it\n- two");
  }

  private static String markdown(String text) {
    return DocComment.plain(text, true);
  }

  private static DocComment traditional(String comment) {
    return DocComment.parse(comment, false);
  }

  private static String plain(String text) {
    return DocComment.plain(text, false);
  }
}
