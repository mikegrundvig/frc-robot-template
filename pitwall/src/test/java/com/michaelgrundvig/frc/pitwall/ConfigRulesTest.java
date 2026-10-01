package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.michaelgrundvig.frc.pitwall.Arms.Arm;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * Rules across configurations, worked through two arms that can collide ({@link Arms}): an intake
 * and a hood, each its own mechanism, clear of each other as committed.
 */
class ConfigRulesTest {
  /** An inch, the clearance the arms keep. */
  private static final double CLEARANCE = 0.0254;

  private final MockTunableBackend dashboard = new MockTunableBackend();
  private final TunableTee tee = new TunableTee(dashboard, () -> null, inputs -> {}, false);
  private final LiveConfig<Arm> intake = Arms.live(Arms.intake());
  private final LiveConfig<Arm> hood = Arms.live(Arms.hood());
  private final ConfigRules rules = new ConfigRules();
  private final AtomicInteger checked = new AtomicInteger();

  @BeforeEach
  void register() {
    rules
        .when(intake, "swing.from", "swing.to")
        .and(hood, "swing.from", "swing.to")
        .check(
            "the intake and the hood",
            (i, h) -> {
              checked.incrementAndGet();
              return Arms.clear(i, h, CLEARANCE);
            });
  }

  @AfterEach
  void reset() {
    TunableRegistry.reset();
  }

  @Test
  void asCommittedTheArmsAreClear() {
    assertThat(Arms.gap(Arms.intake(), Arms.hood())).isGreaterThan(0.1);
    assertThatCode(rules::checkCommitted).doesNotThrowAnyException();
  }

  @Test
  void eachArmAloneMayChangeItsRangeWhereItStaysClear() {
    assertThat(intake.set(Map.of("swing.from", 40.0))).isTrue();
    assertThat(hood.set(Map.of("swing.from", 250.0))).isTrue();
  }

  @Test
  void aChangeThatMakesThemCollideIsRefusedNamingTheOtherArmsRange() {
    assertThat(intake.whyNot(Map.of("swing.from", 20.0)))
        .startsWith("the intake and the hood: the intake (20° to 120°) and the hood (220° to 270°)")
        .contains("they need 2.5 cm");
    assertThat(intake.set(Map.of("swing.from", 20.0))).isFalse();
    assertThat(intake.get()).isEqualTo(Arms.intake());
  }

  @Test
  void thePageIsRefusedTooWithEveryReason() {
    publish();

    var outcome = tee.requestChanges(Map.of("Intake/swing/from", 20));
    tee.update();

    assertThat(outcome.getNow(null).result()).isEqualTo(Outcome.Result.REFUSED);
    assertThat(outcome.getNow(null).detail()).startsWith("the intake and the hood: the intake");
    // The page shows the reason beside every value the rule reads, of both arms.
    assertThat(outcome.getNow(null).reasons())
        .singleElement()
        .satisfies(
            reason -> {
              assertThat(reason.layer()).isEqualTo(Outcome.Layer.RULE);
              assertThat(reason.by()).isEqualTo("the intake and the hood");
              assertThat(reason.paths())
                  .containsExactly(
                      "/Intake/swing/from",
                      "/Intake/swing/to",
                      "/Hood/swing/from",
                      "/Hood/swing/to");
              assertThat(reason.text()).startsWith("the intake (20° to 120°) and the hood");
            });
    assertThat(intake.get()).isEqualTo(Arms.intake());
  }

  /** Swinging the intake further while raising the hood is one valid change; either alone isn't. */
  @Test
  void bothArmsMovingTogetherInOneRequestIsAccepted() {
    publish();
    Map<String, Object> both = new LinkedHashMap<>();
    both.put("Intake/swing/from", 20);
    both.put("Hood/swing/from", 260);

    var outcome = tee.requestChanges(both);
    tee.update();

    assertThat(outcome.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(intake.get().swing()).isEqualTo(new Arms.Swing(20, 120));
    assertThat(hood.get().swing()).isEqualTo(new Arms.Swing(260, 270));
  }

  @Test
  void aDashboardWriteThatCollidesSnapsBackWithTheReason() {
    publish();

    dashboard.setDouble("Intake/swing/from", 20);
    tee.update();

    assertThat(intake.get()).isEqualTo(Arms.intake());
    assertThat(tee.refusals())
        .singleElement()
        .asString()
        .startsWith("/Intake/swing/from = 20.0: the intake and the hood: the intake (20°");
  }

  @Test
  void aChangeToAnythingElseNeverRunsTheRule() {
    publish();
    checked.set(0);

    assertThat(intake.set(Map.of("kP", 6.0))).isTrue();
    var outcome = tee.requestChanges(Map.of("Hood/kP", 3));
    tee.update();

    assertThat(outcome.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(checked).hasValue(0);
  }

  @Test
  void aCommittedPairThatCollidesFailsTheCommittedCheck() {
    var reaching = Arms.live(new Arm("intake", 0, 0, 0.5, new Arms.Swing(20, 120), 4));
    var committed = new ConfigRules();
    committed
        .when(reaching, "swing.from", "swing.to")
        .and(hood, "swing.from", "swing.to")
        .check("the intake and the hood", (i, h) -> Arms.clear(i, h, CLEARANCE));

    assertThatIllegalArgumentException()
        .isThrownBy(committed::checkCommitted)
        .withMessageStartingWith("the intake and the hood: the intake (20° to 120°)");
  }

  /** The rule judged each change when it was made; the log says what was applied. */
  @Test
  void replayAppliesLoggedChangesWithoutRunningTheRuleAgain() {
    var replay =
        new TunableTee(
            new MockTunableBackend(),
            () -> null,
            inputs -> {
              inputs.paths = new String[] {"/Intake/swing/from"};
              inputs.values = new String[] {"20.0"};
            },
            true);
    TunableRegistry.registerBackend("", replay);
    TunableRegistry.publish("Intake", intake);
    TunableRegistry.publish("Hood", hood);
    checked.set(0);

    replay.update();

    assertThat(intake.get().swing().from()).isEqualTo(20);
    assertThat(checked).hasValue(0);
  }

  @Test
  void anInputMustBeATunableValue() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rules.when(intake, "length"))
        .withMessageStartingWith("intake has no tunable value length");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rules.when(intake).and(hood, "swing.form"))
        .withMessageStartingWith("hood has no tunable value swing.form");
  }

  @Test
  void aRuleWithNoValuesNamedRunsForAnyOfItsConfigurations() {
    rules
        .when(intake)
        .check("a short swing", i -> i.swing().to() - i.swing().from() <= 90 ? null : "too long");

    assertThat(intake.whyNot(Map.of("kP", 6.0))).isNull();
    assertThat(intake.whyNot(Map.of("swing.to", 150.0))).isEqualTo("a short swing: too long");
  }

  @Test
  void aRuleThatThrowsRefusesTheChangeAndSaysSo() {
    rules.when(hood, "kP").check("a broken rule", h -> String.valueOf(1 / (int) (h.kP() - 3)));

    assertThat(hood.whyNot(Map.of("kP", 3.0)))
        .startsWith("rule 'a broken rule' failed: java.lang.ArithmeticException");
  }

  @Test
  void rulesHaveDistinctNames() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rules.when(intake).check("the intake and the hood", i -> null))
        .withMessageContaining("already");
  }

  @Test
  void threeOrMoreConfigurationsAreReadFromTheCandidates() {
    var third = Arms.live(new Arm("climber", 1, 0, 0.3, new Arms.Swing(90, 180), 1));
    rules
        .when(intake, "kP")
        .and(hood, "kP")
        .and(third, "kP")
        .check(
            "the gains together",
            c -> c.of(intake).kP() + c.of(hood).kP() + c.of(third).kP() <= 20 ? null : "too high");

    assertThat(third.whyNot(Map.of("kP", 10.0))).isNull();
    assertThat(third.whyNot(Map.of("kP", 15.0))).isEqualTo("the gains together: too high");
  }

  private void publish() {
    TunableRegistry.registerBackend("", tee);
    TunableRegistry.publish("Intake", intake);
    TunableRegistry.publish("Hood", hood);
    tee.update();
  }
}
