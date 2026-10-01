package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.entry;

import java.util.Map;
import org.junit.jupiter.api.Test;

/** Reads what the documentation processor wrote for the test fixtures as they compiled. */
class ParameterDocsTest {
  // Their resources are written by hand (src/test/resources), damaged as a jar's might be. They
  // have no Javadoc, so the processor writes nothing over them.
  record Broken(double x) {}

  record Odd(double a, double b) {}

  @Test
  void aRecordsDocumentationIsItsJavadoc() {
    ParameterDocs gains = ParameterDocs.of(Fixtures.Gains.class);

    assertThat(gains.summary()).isEqualTo("How the drive is controlled.");
    assertThat(gains.text())
        .isEqualTo(
            "How the drive is controlled. Tuned on a running machine, then committed.\n\n"
                + "Start `kP` low.");
    assertThat(gains.params())
        .containsExactly(
            entry("kP", "volts per meter per second of error"),
            entry("lookahead", "how far ahead the drive aims"));
  }

  @Test
  void aRecordWithoutJavadocHasNone() {
    assertThat(ParameterDocs.of(Fixtures.Range.class)).isEqualTo(ParameterDocs.NONE);
    assertThat(ParameterDocs.of(String.class)).as("Java's own").isEqualTo(ParameterDocs.NONE);
  }

  /** The page names a record this way to say whose Javadoc to write. */
  @Test
  void aRecordIsNamedAsCodeInItsPackageWritesIt() {
    record Local(double x) {}

    assertThat(ParameterDocs.nameOf(Fixtures.Gains.class)).isEqualTo("Fixtures.Gains");
    assertThat(ParameterDocs.nameOf(String.class)).isEqualTo("String");
    assertThat(ParameterDocs.nameOf(Local.class))
        .as("declared in a method: with its class")
        .isEqualTo("ParameterDocsTest.Local");
    assertThat(ParameterDocs.nameOf(new Object() {}.getClass()))
        .as("anonymous: its class's")
        .isEqualTo("ParameterDocsTest");
  }

  @Test
  void aDamagedResourceIsNoDocumentation() {
    assertThat(ParameterDocs.of(Broken.class)).isEqualTo(ParameterDocs.NONE);
    assertThat(ParameterDocs.of(Odd.class))
        .as("values of the wrong kind are left out")
        .isEqualTo(new ParameterDocs("", "", Map.of("a", "an a")));
  }

  /** The processor escapes them for JSON, and they come back as written. */
  @Test
  void quotesBackslashesAndSymbolsComeBackAsWritten() {
    assertThat(ParameterDocs.of(Fixtures.Symbols.class).params())
        .containsExactly(
            entry("heading", "the \"zero\" heading, in ° from C:\\field's corner"),
            entry("resistance", "in Ω, or Ω"));
  }

  @Test
  void eachRecordIsReadOnce() {
    assertThat(ParameterDocs.of(Fixtures.Gains.class))
        .isSameAs(ParameterDocs.of(Fixtures.Gains.class));
  }
}
