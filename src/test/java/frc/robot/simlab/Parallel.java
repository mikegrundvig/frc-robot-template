package frc.robot.simlab;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Function;

/** Runs independent simulations on every core. Results come back in input order. */
public final class Parallel {
  private Parallel() {}

  public static <T, R> List<R> map(List<T> inputs, Function<T, R> work) {
    int threads = Runtime.getRuntime().availableProcessors();
    ExecutorService pool = Executors.newFixedThreadPool(threads);
    try {
      List<Future<R>> futures = new ArrayList<>();
      for (T input : inputs) {
        futures.add(pool.submit(() -> work.apply(input)));
      }
      List<R> results = new ArrayList<>();
      for (Future<R> future : futures) {
        results.add(future.get());
      }
      return results;
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IllegalStateException(e);
    } catch (ExecutionException e) {
      throw new IllegalStateException("a simulation run failed", e.getCause());
    } finally {
      pool.shutdownNow();
    }
  }
}
