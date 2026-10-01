package frc.robot.subsystems.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.coprocessor.api.Stamp;
import com.michaelgrundvig.frc.coprocessor.table.Board;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import frc.robot.subsystems.coprocessors.DeployCheck.Asked;
import frc.robot.subsystems.coprocessors.DeployCheck.Finding;
import frc.robot.subsystems.coprocessors.DeployCheck.Verdict;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * The deploy's coprocessor check, its decisions without a network: what each answer means for the
 * deploy.
 */
class DeployCheckTest {
  private static final Computer FRONT =
      new Computer("vision-front", 11, Board.ORANGEPI_5, List.of("front-left"), 5808);
  private static final Computer BACK =
      new Computer("vision-back", 12, Board.ORANGEPI_5, List.of("back-left"), 5808);
  private static final CompiledTable TABLE =
      new CompiledTable(
          new Table(2611, 5808, List.of(FRONT, BACK)), "v2027.1.0", "recipe-1", Map.of());

  /** The stamp this build's image for a computer would carry. */
  private static Stamp stampOf(Computer computer, String version, String recipe) {
    return new Stamp(
        computer.name(),
        2611,
        TABLE.table().ip(computer),
        computer.board().id(),
        computer.cameras(),
        "r1",
        recipe,
        version,
        "",
        "boot",
        "02:00:00:00:00:0b");
  }

  @Test
  void aCoprocessorRunningThisBuildsImagePasses() {
    Finding finding =
        DeployCheck.judge(TABLE, FRONT, new Asked.Stamped(stampOf(FRONT, "v2027.1.0", "recipe-1")));

    assertThat(finding.verdict()).isEqualTo(Verdict.OK);
    assertThat(finding.text()).contains("release r1", "PhotonVision v2027.1.0");
  }

  @Test
  void anotherPhotonVisionFailsNamingBoth() {
    Finding finding =
        DeployCheck.judge(TABLE, FRONT, new Asked.Stamped(stampOf(FRONT, "v2026.3.4", "recipe-1")));

    assertThat(finding.verdict()).isEqualTo(Verdict.FAIL);
    assertThat(finding.text()).contains("v2026.3.4", "v2027.1.0", "10.26.11.11");
  }

  @Test
  void anotherRecipeFails() {
    Finding finding =
        DeployCheck.judge(TABLE, FRONT, new Asked.Stamped(stampOf(FRONT, "v2027.1.0", "recipe-0")));

    assertThat(finding.verdict()).isEqualTo(Verdict.FAIL);
    assertThat(finding.text()).contains("recipe-0", "recipe-1");
  }

  @Test
  void anotherComputersImageFails() {
    Finding finding =
        DeployCheck.judge(TABLE, FRONT, new Asked.Stamped(stampOf(BACK, "v2027.1.0", "recipe-1")));

    assertThat(finding.verdict()).isEqualTo(Verdict.FAIL);
    assertThat(finding.text()).contains("vision-back", "vision-front");
  }

  @Test
  void photonVisionWithoutTheAgentIsntThisImageAndFails() {
    Finding finding =
        DeployCheck.judge(TABLE, FRONT, new Asked.PhotonVisionOnly("connect timed out"));

    assertThat(finding.verdict()).isEqualTo(Verdict.FAIL);
    assertThat(finding.text()).contains("isn't running this repository's image");
  }

  @Test
  void anAnswerThatIsntAStampFails() {
    Finding finding = DeployCheck.judge(TABLE, FRONT, new Asked.Unreadable("not JSON"));

    assertThat(finding.verdict()).isEqualTo(Verdict.FAIL);
  }

  @Test
  void aCoprocessorThatDoesntAnswerOnlyWarns() {
    Finding finding = DeployCheck.judge(TABLE, FRONT, new Asked.Unreachable("connect timed out"));

    assertThat(finding.verdict()).isEqualTo(Verdict.WARN);
    assertThat(finding.text()).contains("deploying anyway");
  }

  @Test
  void everyComputerIsAskedAtItsAddress() {
    List<Finding> findings =
        DeployCheck.check(
            TABLE,
            2611,
            (address, port) -> {
              assertThat(port).isEqualTo(5808);
              return address.equals("10.26.11.11")
                  ? new Asked.Stamped(stampOf(FRONT, "v2027.1.0", "recipe-1"))
                  : new Asked.Unreachable("connect timed out");
            });

    assertThat(findings)
        .extracting(Finding::computer, Finding::verdict)
        .containsExactly(
            org.assertj.core.groups.Tuple.tuple("vision-front", Verdict.OK),
            org.assertj.core.groups.Tuple.tuple("vision-back", Verdict.WARN));
  }

  @Test
  void aTableForAnotherTeamWarns() {
    List<Finding> findings =
        DeployCheck.check(TABLE, 254, (address, port) -> new Asked.Unreachable("no route"));

    assertThat(findings.get(0).verdict()).isEqualTo(Verdict.WARN);
    assertThat(findings.get(0).text()).contains("team 2611's", "team 254's");
  }

  @Test
  void theReportSaysWhetherTheDeployGoesOn() {
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    PrintStream print = new PrintStream(out, true, StandardCharsets.UTF_8);

    boolean ok =
        DeployCheck.report(
            List.of(
                new Finding("vision-front", Verdict.OK, "fine"),
                new Finding("vision-back", Verdict.WARN, "nothing answers")),
            print);
    boolean failed =
        DeployCheck.report(List.of(new Finding("vision-front", Verdict.FAIL, "wrong")), print);
    boolean empty = DeployCheck.report(List.of(), print);
    boolean silent =
        DeployCheck.report(List.of(new Finding("vision-front", Verdict.WARN, "nothing")), print);

    assertThat(ok).isTrue();
    assertThat(failed).isFalse();
    assertThat(empty).isTrue();
    assertThat(silent).isTrue();
    assertThat(out.toString(StandardCharsets.UTF_8))
        .contains(
            "Coprocessor check: OK vision-front: fine",
            "Coprocessor check: WARN vision-back: nothing answers",
            "Coprocessor check: 1 answered, running this build's image.",
            "Coprocessor check: FAILED. Nothing was deployed.",
            "the table has no coprocessors",
            "none answered, so none was compared with this build");
  }

  @Test
  void askingNobodyIsUnreachable() throws Exception {
    // A closed port on this computer: refused at once, and nothing on PhotonVision's port either.
    int free;
    try (ServerSocket socket = new ServerSocket(0, 1, InetAddress.getLoopbackAddress())) {
      free = socket.getLocalPort();
    }

    Asked asked = DeployCheck.ask("127.0.0.1", free);

    assertThat(asked).isInstanceOf(Asked.Unreachable.class);
  }
}
