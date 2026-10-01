package com.michaelgrundvig.frc.pitwall.docs;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.Map;
import org.junit.jupiter.api.Test;

/** Doc comments as the compiler gives them: each line without its leading star. */
class DocCommentTest {
  @Test
  void theSummaryIsTheFirstSentence() {
    assertThat(DocComment.parse(" Gains. Tuned live.\n").summary()).isEqualTo("Gains.");
    assertThat(DocComment.parse(" Gains, version 1.5 of them\n\n<p>More.").summary())
        .as("a period inside a number doesn't end it, and a paragraph does")
        .isEqualTo("Gains, version 1.5 of them");
    assertThat(DocComment.parse(" @param x a value\n").summary()).isEmpty();
  }

  @Test
  void codeKeepsItsBracesAndItsAtSigns() {
    DocComment doc =
        DocComment.parse(
            " Built as {@code new double[] {1, 2}} or {@code\n @Config record X()}.\n"
                + " @param x the {@code {}} value\n");

    assertThat(doc.text()).isEqualTo("Built as `new double[] {1, 2}` or `@Config record X()`.");
    assertThat(doc.params()).containsExactly(Map.entry("x", "the `{}` value"));
  }

  @Test
  void aLinkShowsItsLabelOrWhatItNames() {
    assertThat(DocComment.plain("{@link frc.robot.lib.config.SwerveConfig.Gains}"))
        .isEqualTo("SwerveConfig.Gains");
    assertThat(DocComment.plain("{@link Limits#between(double, double)}"))
        .isEqualTo("between(double, double)");
    assertThat(DocComment.plain("{@linkplain Limits#between(double, double) its limits}"))
        .isEqualTo("its limits");
    assertThat(DocComment.plain("{@link #NONE}")).isEqualTo("NONE");
  }

  @Test
  void otherMarkupIsDropped() {
    assertThat(DocComment.plain("a <a href=\"https://wpilib.org\">WPILib</a> link<br>next"))
        .isEqualTo("a WPILib link next");
    assertThat(DocComment.plain("x &lt; y &#38; y &#x3E; z, &unknown; a < b, <!-- hidden -->c"))
        .isEqualTo("x < y & y > z, &unknown; a < b, c");
    assertThat(DocComment.plain("{@literal <p>}{@inheritDoc}")).isEqualTo("<p>");
  }

  @Test
  void listsAndParagraphsKeepTheirShape() {
    assertThat(DocComment.plain("Kinds:<ol><li>one<li>two</li></ol>Then <P>more</P>"))
        .isEqualTo("Kinds:\n- one\n- two\n\nThen\n\nmore");
  }
}
