package frc.robot.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;

import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.VisionDecision;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.SimulationTest;
import frc.robot.sim.DriveSimHarness;
import frc.robot.sim.FakeCamera;
import frc.robot.sim.SensorErrors;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * The pose estimator's behavior, recorded: a seeded session (driving, a misread tag, a collision,
 * an unfelt shove, a tilt) whose estimate, loop by loop, must match {@value #GOLDEN}. The session
 * repeats exactly, so any difference is a change in what the estimator does: the safety net for
 * restructuring it without changing it.
 *
 * <p>When a change to the estimator (or to the simulator under it) is meant to change its behavior,
 * rerun this test with the environment variable {@code GOLDEN_UPDATE=1} to record the new behavior,
 * and commit the file with the change, saying why it moved.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class EstimatorGoldenSimTest {
  static final String GOLDEN = "estimator-golden.csv";

  /** Close enough for the same arithmetic on another processor; far below any real change. */
  private static final double TOLERANCE = 1e-6;

  @Test
  void theEstimatorBehavesAsRecorded() throws IOException {
    List<String> actual = session().rows();

    if ("1".equals(System.getenv("GOLDEN_UPDATE"))) {
      Path file =
          Path.of(System.getProperty("frc.projectDir", "."), "src", "test", "resources")
              .resolve(getClass().getPackageName().replace('.', '/'))
              .resolve(GOLDEN);
      Files.createDirectories(file.getParent());
      Files.writeString(file, String.join("\n", actual) + "\n", StandardCharsets.UTF_8);
      return;
    }
    List<String> expected = recorded();
    for (int i = 0; i < Math.min(expected.size(), actual.size()); i++) {
      if (!matches(expected.get(i), actual.get(i))) {
        fail(
            "The estimate differs from the recording at row %d (loop,x,y,heading,mode,accepted,"
                + "rejected):%n  recorded %s%n  now      %s%nIf the change is meant, record it"
                + " again with GOLDEN_UPDATE=1 (see this test's documentation).",
            i, expected.get(i), actual.get(i));
      }
    }
    assertThat(actual).as("rows").hasSameSizeAs(expected);
  }

  /**
   * The misread tag, 1.5 m off and claiming 3 cm, comes as the robot reverses at full stick, its
   * drive motors at their current limit: wheels that grip, so the estimate stays sure of them and
   * the gate rejects it.
   */
  @Test
  void theMisreadTagIsRejected() {
    List<VisionDecision> misread =
        session().decisions().stream().filter(decision -> decision.innovationMeters() > 1).toList();

    assertThat(misread).hasSize(1);
    assertThat(misread.get(0).accepted()).isFalse();
  }

  /** What the session did: a row per loop, and what became of every camera measurement. */
  private record Session(List<String> rows, List<VisionDecision> decisions) {}

  /** The session, one row per loop: the loop, the pose, the mode, and the measurement counts. */
  private static Session session() {
    List<String> rows = new ArrayList<>();
    rows.add("loop,x,y,heading,mode,accepted,rejected");
    try (DriveSimHarness robot = new DriveSimHarness(Robots.SIM, SensorErrors.TYPICAL)) {
      FakeCamera camera = new FakeCamera(robot.simulation, 0.03, 0.04, 30, 3);
      robot.watchWith(camera);
      int[] loop = {0};
      robot.eachLoop(() -> rows.add(row(++loop[0], robot.estimator.estimate())));

      robot.drive(2, 1, 0, 0);
      robot.drive(1.5, 0, 1, 0.5);
      camera.lieOnce(new Transform2d(1.5, 0, Rotation2d.ZERO)); // a misread tag
      robot.drive(1.5, -1, 0, 0);
      robot.simulation.hit(new Transform2d(0, 0.4, Rotation2d.ZERO)); // a collision, felt
      robot.drive(1.5, 0.5, -0.5, -0.8);
      robot.simulation.shove(new Transform2d(0.3, 0, Rotation2d.ZERO)); // pushed, unfelt
      robot.drive(1.5, 0, 0, 0);
      robot.simulation.tilt(Math.toRadians(-10), Math.toRadians(4));
      robot.drive(1, 0.6, 0, 0);
      robot.simulation.tilt(0, 0);
      robot.idle(1);
      return new Session(rows, List.copyOf(robot.decisions));
    }
  }

  private static String row(int loop, LocalizationEstimate estimate) {
    return String.format(
        Locale.ROOT,
        "%d,%.9f,%.9f,%.9f,%s,%d,%d",
        loop,
        estimate.pose().getX(),
        estimate.pose().getY(),
        estimate.pose().getRotation().getRadians(),
        estimate.mode(),
        estimate.health().accepted(),
        estimate.health().rejected());
  }

  private List<String> recorded() throws IOException {
    try (InputStream in = getClass().getResourceAsStream(GOLDEN)) {
      if (in == null) {
        return fail("No recording: run this test with GOLDEN_UPDATE=1 to make " + GOLDEN);
      }
      return new String(in.readAllBytes(), StandardCharsets.UTF_8).lines().toList();
    }
  }

  /** The same row: numbers within the tolerance, everything else exactly. */
  private static boolean matches(String expected, String actual) {
    String[] want = expected.split(",");
    String[] got = actual.split(",");
    if (want.length != got.length) {
      return false;
    }
    for (int i = 0; i < want.length; i++) {
      if (want[i].equals(got[i])) {
        continue;
      }
      try {
        if (Math.abs(Double.parseDouble(want[i]) - Double.parseDouble(got[i])) > TOLERANCE) {
          return false;
        }
      } catch (NumberFormatException e) {
        return false;
      }
    }
    return true;
  }
}
