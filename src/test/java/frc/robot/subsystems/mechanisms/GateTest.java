package frc.robot.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Milliseconds;

import frc.robot.RobotSim;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;

/**
 * Gates on their own: worked out once a loop, before what the scheduler runs, with their reasons.
 */
@ExtendWith(RobotSim.class)
class GateTest {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();

  private void loop() {
    RobotSim.stepClock();
    scheduler.run();
  }

  @Test
  void aGateHoldsWhatItsConditionSaysFromTheStart() {
    boolean[] clear = {true};
    Gate gate = Gate.of(scheduler, "intake clear", () -> clear[0]);

    boolean first = gate.getAsBoolean();
    clear[0] = false;
    boolean beforeTheLoop = gate.getAsBoolean();
    loop();

    assertThat(first).as("worked out as it's made").isTrue();
    assertThat(beforeTheLoop).as("then once a loop").isTrue();
    assertThat(gate.getAsBoolean()).isFalse();
    assertThat(gate.name()).isEqualTo("intake clear");
    assertThat(gate.why()).isEqualTo("intake clear is false");
  }

  /**
   * Within 2° to become true, and it stays true out to 4°: an error that wavers doesn't flicker.
   */
  @Test
  void withinBecomesTrueInsideEnterAndStaysTrueInsideStay() {
    double[] error = {5};
    Gate aimed = Gate.within(scheduler, "aimed", () -> error[0], 2, 4);
    List<Boolean> seen = new ArrayList<>();

    for (double each : new double[] {3, 1.9, 3.5, -3.9, 4.1, 3, -1.5}) {
      error[0] = each;
      loop();
      seen.add(aimed.getAsBoolean());
    }

    assertThat(seen).containsExactly(false, true, true, true, false, false, true);
  }

  @Test
  void allIsBlockedByTheTermsThatAreFalse() {
    boolean[] fast = {false};
    double[] error = {10};
    Gate atSpeed = Gate.of(scheduler, "shooter at goal", () -> fast[0]);
    Gate aimed = Gate.within(scheduler, "aimed", () -> error[0], 2, 4);
    Gate ready = Gate.all("ready to shoot", atSpeed, aimed);

    loop();
    List<String> both = ready.blocking();
    fast[0] = true;
    loop();
    List<String> one = ready.blocking();
    error[0] = 0;
    loop();

    assertThat(both).containsExactly("shooter at goal", "aimed");
    assertThat(one).containsExactly("aimed");
    assertThat(ready.getAsBoolean()).isTrue();
    assertThat(ready.blocking()).isEmpty();
  }

  /**
   * Debounced on the falling edge, a gate rides out a dip shorter than its time, falls after one
   * longer, and holds nothing until it first has held: a fresh "ready to shoot" isn't true for its
   * first 80 ms.
   */
  @Test
  void aFallingDebounceRidesOutADipButNotAFall() {
    boolean[] ready = {false};
    Gate gate =
        Gate.of(scheduler, "ready", () -> ready[0])
            .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING);

    loop();
    boolean fresh = gate.getAsBoolean();
    ready[0] = true;
    loop();
    boolean up = gate.getAsBoolean();
    ready[0] = false;
    loop();
    loop();
    loop();
    boolean dipping = gate.getAsBoolean(); // 40 ms in: still true
    List<String> dippingWhy = gate.blocking();
    for (int i = 0; i < 3; i++) {
      loop();
    }

    assertThat(fresh).isFalse();
    assertThat(up).isTrue();
    assertThat(dipping).isTrue();
    assertThat(dippingWhy).isEmpty();
    assertThat(gate.getAsBoolean()).as("false for 100 ms: fallen").isFalse();
    assertThat(gate.name()).isEqualTo("ready");
  }

  @Test
  void aRisingDebounceWaitsAndSaysItsSettling() {
    boolean[] ready = {false};
    Gate gate =
        Gate.of(scheduler, "ready", () -> ready[0])
            .debounce(Milliseconds.of(60), Debouncer.DebounceType.RISING);

    ready[0] = true;
    loop();
    boolean early = gate.getAsBoolean();
    List<String> why = gate.blocking();
    for (int i = 0; i < 4; i++) {
      loop();
    }

    assertThat(early).isFalse();
    assertThat(why).containsExactly("ready settling");
    assertThat(gate.getAsBoolean()).isTrue();
  }

  /** A gate binds and awaits like any trigger: a command waits on it, and a binding runs on it. */
  @Test
  void itBindsAndAwaitsAsATrigger() {
    boolean[] ready = {false};
    Gate gate = Gate.of(scheduler, "ready", () -> ready[0]);
    int[] ran = {0};
    boolean[] waited = {false};
    gate.onTrue(Command.noRequirements(co -> ran[0]++).named("On ready"));
    scheduler.schedule(
        Command.noRequirements(
                co -> {
                  co.waitUntil(gate);
                  waited[0] = true;
                })
            .named("Wait"));

    loop();
    loop();
    boolean before = waited[0];
    ready[0] = true;
    loop();

    assertThat(before).isFalse();
    assertThat(waited[0]).isTrue();
    assertThat(ran[0]).isEqualTo(1);
  }

  @Test
  void badBandsAndEmptyGatesAreRefused() {
    assertThatThrownBy(() -> Gate.within(scheduler, "aimed", () -> 0, 4, 2))
        .hasMessage(
            "aimed: enter within a positive band, and stay within at least it, not 4.0 and 2.0");
    assertThatThrownBy(() -> Gate.all("nothing")).hasMessage("nothing has no terms");
    Gate other = Gate.of(Scheduler.createIndependentScheduler(), "elsewhere", () -> true);
    assertThatThrownBy(() -> Gate.all("mixed", Gate.of(scheduler, "here", () -> true), other))
        .hasMessage("mixed's terms run on different schedulers");
  }
}
