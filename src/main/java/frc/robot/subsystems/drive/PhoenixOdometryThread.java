package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Timestamp;
import frc.robot.lib.localization.Timestamps;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.wpilib.system.RobotController;

/**
 * Reads the drivetrain's odometry signals as fast as the devices publish them (250 Hz), on its own
 * thread, so odometry sees every turn of the wheels whatever the main loop is doing. It waits until
 * every signal has a new value, then records them together, stamped with when they were measured:
 * now, less the signals' average time in transit. The main loop drains the readings every loop
 * through the IO layer, so they're logged, and a replay sees exactly what the robot saw.
 *
 * <p>On Systemcore, waiting works on every CAN bus and Phoenix shares the robot's clock (Phoenix
 * 26.70). Register every signal before starting the thread.
 */
final class PhoenixOdometryThread extends Thread {
  /** How often the drivetrain's devices publish odometry signals, and this thread reads them. */
  static final double FREQUENCY_HZ = 250;

  /** Readings held for the main loop: four loops' worth, in case one runs late. */
  private static final int CAPACITY = 20;

  /** A signal's time in transit is trusted up to this; longer means it never arrived. */
  private static final double MAX_LATENCY_SECONDS = 0.1;

  private final ReentrantLock lock = new ReentrantLock();
  private final List<Queue<Double>> values = new ArrayList<>();
  private final List<Queue<Long>> timestamps = new ArrayList<>();
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];

  PhoenixOdometryThread() {
    setName("PhoenixOdometry");
    setDaemon(true);
  }

  /** Reads a signal from now on; its readings arrive on the returned queue, oldest first. */
  Queue<Double> register(BaseStatusSignal signal) {
    lock.lock();
    try {
      BaseStatusSignal[] more = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, more, 0, signals.length);
      more[signals.length] = signal;
      signals = more;
      Queue<Double> queue = new ArrayBlockingQueue<>(CAPACITY);
      values.add(queue);
      return queue;
    } finally {
      lock.unlock();
    }
  }

  /**
   * A queue of every reading's time, in nanoseconds on the robot's clock: one for each device's IO,
   * each drained with that device's readings.
   */
  Queue<Long> timestamps() {
    lock.lock();
    try {
      Queue<Long> queue = new ArrayBlockingQueue<>(CAPACITY);
      timestamps.add(queue);
      return queue;
    } finally {
      lock.unlock();
    }
  }

  /**
   * Hold this while draining the readings, all devices together, so every device's readings stay in
   * step: the thread records only between drains.
   */
  Lock lock() {
    return lock;
  }

  @Override
  public void run() {
    while (!isInterrupted()) {
      BaseStatusSignal[] reading;
      lock.lock();
      try {
        reading = signals;
      } finally {
        lock.unlock();
      }
      if (reading.length == 0) {
        try {
          Thread.sleep(Math.round(1000 / FREQUENCY_HZ));
        } catch (InterruptedException e) {
          return;
        }
        continue;
      }
      // Waits until every signal has a new value, or two periods pass (a device is missing: its
      // value stays as it was, and its IO reports it disconnected).
      BaseStatusSignal.waitForAll(2 / FREQUENCY_HZ, reading);
      record(reading);
    }
  }

  private void record(BaseStatusSignal[] reading) {
    lock.lock();
    try {
      double latency = 0;
      int timed = 0;
      for (BaseStatusSignal signal : reading) {
        Timestamp stamp = signal.getTimestamp();
        if (stamp.isValid() && stamp.getLatency() >= 0) {
          latency += Math.min(stamp.getLatency(), MAX_LATENCY_SECONDS);
          timed++;
        }
      }
      long measured =
          RobotController.getTime() - (timed == 0 ? 0 : Timestamps.fromSeconds(latency / timed));
      for (int i = 0; i < reading.length; i++) {
        values.get(i).offer(reading[i].getValueAsDouble());
      }
      for (Queue<Long> queue : timestamps) {
        queue.offer(measured);
      }
    } finally {
      lock.unlock();
    }
  }
}
