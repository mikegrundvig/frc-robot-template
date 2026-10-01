package com.michaelgrundvig.frc.pitwall.experiment;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;

/**
 * Experiments on a {@link Lag}, whose runs a test drives by hand, a step each loop, as their
 * commands would be driven: for Pitwall's tests outside this package, where there's no command
 * scheduler (it needs WPILib's native HAL, which only the robot program's tests load).
 */
public final class ByHand {
  /** The one experiment: a step to 90° and back on the lag. */
  public static final String STEP = "Step to 90° and back";

  private final Lag lag = new Lag();
  private final Experiments experiments;
  private @Nullable Run driven;

  /**
   * @param wallClock when requests arrive, in nanoseconds, for the deadman
   */
  public ByHand(LongSupplier wallClock) {
    experiments =
        new Experiments(
                new Experiments.Robot("test", true, "abc12345 on main"),
                inputs -> {},
                wallClock,
                false,
                run ->
                    new Actions.Routine() {
                      @Override
                      public @Nullable String start() {
                        driven = run;
                        return null;
                      }

                      @Override
                      public boolean isRunning() {
                        return run.state().phase() == Run.Phase.RUNNING;
                      }

                      @Override
                      public void cancel() {
                        run.cancel("Canceled: another command needed Arm");
                      }
                    },
                ended -> {})
            .add(new Experiment(STEP, lag.rig(), Lag.STEP_AND_BACK));
  }

  public Experiments experiments() {
    return experiments;
  }

  /** The lag's configuration, whose gains the experiment's rig reads: kP and kD. */
  public LiveConfig<?> config() {
    return lag.config;
  }

  /** The scheduler's part of a loop: the lag moves on, and the run takes its step. */
  public void step() {
    long now = lag.nextLoop();
    Run run = driven;
    if (run != null) {
      run.step(now);
    }
  }
}
