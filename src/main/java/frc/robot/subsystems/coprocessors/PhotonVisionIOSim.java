package frc.robot.subsystems.coprocessors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.photonvision.PhotonVersion;

/**
 * PhotonVision's version as the simulated coprocessors would publish it: each publishes its own as
 * its PhotonVision starts (on its first answer, after a reboot, or once its version changes), in
 * the table's order when several start together. Like the real topic, it holds whichever spoke
 * last.
 */
public final class PhotonVisionIOSim implements PhotonVisionIO {
  private final List<CoprocessorIOSim> coprocessors;

  /** What each published last, and in which boot. */
  private final Map<CoprocessorIOSim, String> published = new HashMap<>();

  private final Map<CoprocessorIOSim, Integer> boots = new HashMap<>();
  private String reported = "";

  public PhotonVisionIOSim(List<CoprocessorIOSim> coprocessors) {
    this.coprocessors = List.copyOf(coprocessors);
  }

  @Override
  public void updateInputs(PhotonVisionIOInputs inputs) {
    inputs.library = PhotonVersion.versionString;
    List<String> now = new ArrayList<>();
    for (CoprocessorIOSim coprocessor : coprocessors) {
      if (!coprocessor.isAnswering()) {
        continue;
      }
      String version = coprocessor.publishedVersion();
      Integer boot = boots.get(coprocessor);
      if (!version.equals(published.get(coprocessor))
          || boot == null
          || boot != coprocessor.boot()) {
        published.put(coprocessor, version);
        boots.put(coprocessor, coprocessor.boot());
        now.add(version);
        reported = version;
      }
    }
    inputs.published = now.toArray(String[]::new);
    inputs.reported = reported;
  }
}
