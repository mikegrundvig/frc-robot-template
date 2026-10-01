package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.entry;

import java.util.Map;
import org.junit.jupiter.api.Test;

/** Reads what the documentation processor wrote for the test fixtures as they compiled. */
class ParameterDocsTest {
  /**
   * What a configuration writes that JSON must escape, or that isn't ASCII.
   *
   * @param heading the "zero" heading, in ° from C:\field's corner
   * @param resistance in Ω, or &Omega;
   */
  record Symbols(double heading, double resistance) {}

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

  /** The record that declares a value is found through the records that hold it. */
  @Test
  void aValueIsFoundByItsPathAsLiveConfigNamesIt() {
    assertThat(ParameterDocs.param(Fixtures.Machine.class, "drive.gains.kP"))
        .isEqualTo("volts per meter per second of error");
    assertThat(ParameterDocs.param(Fixtures.Machine.class, "drive.limits.current"))
        .isEqualTo("the most current through the motor");
    assertThat(ParameterDocs.param(Fixtures.Gains.class, "lookahead"))
        .isEqualTo("how far ahead the drive aims");
  }

  @Test
  void whatsMissingIsEmptyNeverAnError() {
    assertThat(ParameterDocs.param(Fixtures.Machine.class, "drive.limits.maxSpeed"))
        .as("no @param")
        .isEmpty();
    assertThat(ParameterDocs.param(Fixtures.Machine.class, "travel.min"))
        .as("no Javadoc")
        .isEmpty();
    assertThat(ParameterDocs.param(Fixtures.Machine.class, "drive.wheels.kP"))
        .as("no such value")
        .isEmpty();
    assertThat(ParameterDocs.param(Fixtures.Machine.class, "name.length"))
        .as("not a record")
        .isEmpty();
    assertThat(ParameterDocs.of(Fixtures.Range.class)).isEqualTo(ParameterDocs.NONE);
    assertThat(ParameterDocs.of(String.class)).as("Java's own").isEqualTo(ParameterDocs.NONE);
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
    assertThat(ParameterDocs.of(Symbols.class).params())
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
