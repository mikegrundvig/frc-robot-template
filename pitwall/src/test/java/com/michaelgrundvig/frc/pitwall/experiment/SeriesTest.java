package com.michaelgrundvig.frc.pitwall.experiment;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class SeriesTest {
  @Test
  void keepsSamplesInOrder() {
    var series = new Series(10);

    assertThat(series.add(1_000, 0.5)).isTrue();
    assertThat(series.add(2_000, 0.75)).isTrue();

    assertThat(series.count()).isEqualTo(2);
    assertThat(series.time(1)).isEqualTo(2_000);
    assertThat(series.value(1)).isEqualTo(0.75);
    assertThat(series.dropped()).isZero();
    assertThat(series.full()).isFalse();
  }

  @Test
  void aBadReadingCostsThatReadingAndIsCounted() {
    var series = new Series(10);
    series.add(1_000, 1.0);

    assertThat(series.add(2_000, Double.NaN)).as("not a number").isFalse();
    assertThat(series.add(2_000, Double.POSITIVE_INFINITY)).as("infinite").isFalse();
    assertThat(series.add(1_000, 2.0)).as("not later than the last").isFalse();
    assertThat(series.add(500, 2.0)).as("earlier than the last").isFalse();
    assertThat(series.add(2_000, 2.0)).as("a good one after").isTrue();

    assertThat(series.count()).isEqualTo(2);
    assertThat(series.dropped()).isEqualTo(4);
  }

  @Test
  void stopsWhenFullAndSaysSo() {
    var series = new Series(2);
    series.add(1, 1);
    series.add(2, 2);

    assertThat(series.add(3, 3)).isFalse();

    assertThat(series.count()).isEqualTo(2);
    assertThat(series.full()).isTrue();
    assertThat(series.dropped()).isEqualTo(1);
    assertThat(series.capacity()).isEqualTo(2);
  }

  @Test
  void readsOnlyWhatHasBeenRecorded() {
    var series = new Series(10);
    series.add(1, 1);

    assertThatThrownBy(() -> series.value(1)).isInstanceOf(IndexOutOfBoundsException.class);
    assertThatThrownBy(() -> series.time(-1)).isInstanceOf(IndexOutOfBoundsException.class);
  }

  @Test
  void findsThePeakMagnitudeWithinASpan() {
    var series = new Series(10);
    series.add(1, 2.0);
    series.add(2, -7.5);
    series.add(3, 5.0);
    series.add(4, 9.0);

    assertThat(series.peakMagnitude(1, 3).getAsDouble()).isEqualTo(7.5);
    assertThat(series.peakMagnitude(5, 9)).as("nothing in the span").isEmpty();
  }

  @Test
  void needsRoomForASample() {
    assertThatIllegalArgumentException().isThrownBy(() -> new Series(0));
  }

  @Test
  void aReaderOnAnotherThreadSeesOnlyWholeSamples() throws InterruptedException {
    int samples = 200_000;
    var series = new Series(samples);
    var problem = new AtomicReference<String>();
    Thread reader =
        new Thread(
            () -> {
              while (series.count() < samples && problem.get() == null) {
                int n = series.count();
                for (int i = Math.max(0, n - 64); i < n; i++) {
                  if (series.value(i) != series.time(i) * 0.5) {
                    problem.set("sample " + i + " was read half written");
                  }
                }
              }
            });
    reader.start();
    for (int i = 1; i <= samples; i++) {
      series.add(i, i * 0.5);
    }
    reader.join(10_000);

    assertThat(problem.get()).isNull();
  }
}
