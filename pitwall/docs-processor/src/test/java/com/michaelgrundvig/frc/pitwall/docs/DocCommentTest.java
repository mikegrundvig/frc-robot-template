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
    assertThat(plain("{@link frc.robot.lib.config.SwerveConfig.Gains}"))
        .isEqualTo("SwerveConfig.Gains");
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
    assertThat(plain("a&#7;b&#x1F;c")).as("control characters are dropped").isEqualTo("abc");
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

  private static DocComment traditional(String comment) {
    return DocComment.parse(comment, false);
  }

  private static String plain(String text) {
    return DocComment.plain(text, false);
  }
}
