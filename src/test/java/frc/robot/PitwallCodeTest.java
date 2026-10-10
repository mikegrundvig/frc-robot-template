package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.Status;
import com.michaelgrundvig.frc.runtime.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/** Pitwall's status says which code is running, from the build's generated constants. */
@ExtendWith(RobotSim.class)
class PitwallCodeTest {
  @Test
  void theStatusShowsTheCommitAndBranchTheProgramWasBuiltFrom() {
    try (Pitwall pitwall = Pitwall.start("test", 0)) {
      pitwall.status().requestRead(); // as the page's GET does: nothing is read unless asked
      pitwall.periodic();

      Status.Item code =
          pitwall.status().view().stream()
              .filter(item -> item.name().equals("Robot/Code"))
              .findFirst()
              .orElseThrow();
      assertThat(code.value().toString()).contains(" on " + BuildConstants.GIT_BRANCH);
    }
  }
}
