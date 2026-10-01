package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.api.AgentApi;
import com.michaelgrundvig.frc.coprocessor.api.Stamp;
import com.michaelgrundvig.frc.coprocessor.json.JsonException;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import java.io.IOException;
import java.io.PrintStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * The check {@code deploy} runs first: asks each coprocessor in the table for its stamp, within a
 * second, and compares it with this build. A coprocessor that answers with another PhotonVision,
 * image recipe, name, team, or address than this build's fails the deploy, as does one that answers
 * on PhotonVision's port but not the agent's (it isn't running this repository's image). One that
 * doesn't answer at all only warns, so a robot can be deployed with its vision off.
 *
 * <p>Run by Gradle ({@code ./gradlew coprocessorCheck}, which {@code deploy} depends on), on the
 * laptop, never on the robot. {@code -PskipCoprocessorCheck} skips it, and says so.
 */
public final class DeployCheck {
  /** How long each coprocessor has to connect, and then to answer. */
  static final double TIMEOUT_SECONDS = 1;

  /** What a coprocessor's stamp can hold, at most. */
  private static final int MAX_STAMP_BYTES = 16 * 1024;

  private DeployCheck() {}

  /** What became of asking one coprocessor. */
  sealed interface Asked {
    /** Its agent answered with its stamp. */
    record Stamped(Stamp stamp) implements Asked {}

    /** Its agent answered, but not with a stamp this build can read. */
    record Unreadable(String why) implements Asked {}

    /** Its agent didn't answer, but PhotonVision's port did: not this repository's image. */
    record PhotonVisionOnly(String why) implements Asked {}

    /** Nothing answered: off, unplugged, or not on this network. */
    record Unreachable(String why) implements Asked {}
  }

  /** Asks a coprocessor: the network, or a test's stand-in. */
  @FunctionalInterface
  interface Asker {
    Asked ask(String address, int agentPort);
  }

  /** How a coprocessor stands against this build. */
  enum Verdict {
    OK,
    WARN,
    FAIL
  }

  /**
   * One coprocessor's verdict, and why.
   *
   * @param computer its name
   * @param verdict whether the deploy may go on
   * @param text what was found
   */
  record Finding(String computer, Verdict verdict, String text) {
    @Override
    public String toString() {
      return verdict + " " + computer + ": " + text;
    }
  }

  /**
   * One coprocessor's verdict, from what asking it found.
   *
   * @param table the table compiled into this build, with what it expects
   * @param computer the coprocessor, from that table
   * @param asked what asking it found
   */
  static Finding judge(CompiledTable table, Computer computer, Asked asked) {
    String name = computer.name();
    String at = table.table().ip(computer);
    return switch (asked) {
      case Asked.Stamped(Stamp stamp) -> {
        List<String> mismatches = table.mismatches(computer, stamp);
        yield mismatches.isEmpty()
            ? new Finding(
                name,
                Verdict.OK,
                "release "
                    + stamp.release()
                    + ", PhotonVision "
                    + stamp.photonvisionVersion()
                    + ", as this build's")
            : new Finding(
                name,
                Verdict.FAIL,
                "the wrong image at "
                    + at
                    + ": "
                    + String.join("; ", mismatches)
                    + ". Flash it with this build's release, or deploy with"
                    + " -PskipCoprocessorCheck to leave it as it is");
      }
      case Asked.Unreadable(String why) ->
          new Finding(
              name,
              Verdict.FAIL,
              "its agent at "
                  + at
                  + " answered, but not with a stamp this build reads ("
                  + why
                  + "): an image from another version of this repository?");
      case Asked.PhotonVisionOnly(String why) ->
          new Finding(
              name,
              Verdict.FAIL,
              "PhotonVision answers at "
                  + at
                  + ", but its health agent doesn't ("
                  + why
                  + "): it isn't running this repository's image. Flash it with this build's"
                  + " release");
      case Asked.Unreachable(String why) ->
          new Finding(
              name,
              Verdict.WARN,
              "nothing answers at "
                  + at
                  + " ("
                  + why
                  + "): deploying anyway, with its cameras off until it answers");
    };
  }

  /**
   * Every coprocessor's verdict, and one for the table itself when its team isn't the one being
   * deployed to.
   *
   * @param deployTeam the team number the deploy goes to; 0 or less when unknown
   */
  static List<Finding> check(CompiledTable table, int deployTeam, Asker asker) {
    List<Finding> findings = new ArrayList<>();
    int team = table.table().team();
    if (deployTeam > 0 && team != deployTeam) {
      findings.add(
          new Finding(
              "the table",
              Verdict.WARN,
              "coprocessors/coprocessors.yaml is team "
                  + team
                  + "'s, and this deploy is to team "
                  + deployTeam
                  + "'s robot: set its team to ask the right addresses"));
    }
    for (Computer computer : table.table().computers()) {
      Asked asked = asker.ask(table.table().ip(computer), computer.agentPort());
      findings.add(judge(table, computer, asked));
    }
    return findings;
  }

  /** Asks a coprocessor over the network: its agent's stamp, else whether PhotonVision answers. */
  static Asked ask(String address, int agentPort) {
    AgentHttp agent =
        new AgentHttp(address, agentPort, TIMEOUT_SECONDS, TIMEOUT_SECONDS, MAX_STAMP_BYTES);
    byte[] body;
    try {
      body = agent.get(AgentApi.STAMP);
    } catch (IOException noAgent) {
      String why = CoprocessorPoller.why(noAgent);
      try (Socket photonVision = new Socket()) {
        photonVision.connect(
            new InetSocketAddress(address, Coprocessors.PHOTONVISION_PORT),
            (int) (TIMEOUT_SECONDS * 1000));
        return new Asked.PhotonVisionOnly(why);
      } catch (IOException nothing) {
        return new Asked.Unreachable(why);
      }
    }
    try {
      return new Asked.Stamped(Stamp.parse(new String(body, StandardCharsets.UTF_8)));
    } catch (JsonException | IllegalArgumentException e) {
      return new Asked.Unreadable(String.valueOf(e.getMessage()));
    }
  }

  /**
   * Prints every finding, and whether the deploy may go on.
   *
   * @return whether it may: nothing failed
   */
  static boolean report(List<Finding> findings, PrintStream out) {
    if (findings.isEmpty()) {
      out.println("Coprocessor check: the table has no coprocessors");
      return true;
    }
    boolean failed = false;
    for (Finding finding : findings) {
      out.println("Coprocessor check: " + finding);
      failed |= finding.verdict() == Verdict.FAIL;
    }
    out.println(
        failed
            ? "Coprocessor check: FAILED. Nothing was deployed."
            : "Coprocessor check: the coprocessors that answered run this build's image.");
    return !failed;
  }

  /**
   * Checks every coprocessor in the table compiled into this build; exits 1 if the deploy must
   * stop.
   *
   * @param args the team number deployed to, if known
   */
  public static void main(String[] args) {
    int team = args.length > 0 && args[0].matches("\\d{1,5}") ? Integer.parseInt(args[0]) : 0;
    CompiledTable table = CompiledTable.load();
    if (!report(check(table, team, DeployCheck::ask), System.out)) {
      System.exit(1);
    }
  }
}
