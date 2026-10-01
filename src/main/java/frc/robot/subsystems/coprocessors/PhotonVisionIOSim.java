package frc.robot.subsystems.coprocessors;

import java.util.List;
import org.photonvision.PhotonVersion;

/**
 * PhotonVision's version as the simulated coprocessors would publish it: this build's, unless one
 * runs another ({@link CoprocessorIOSim#wrongVersion}). Like the real topic, it holds whichever
 * coprocessor spoke last: one running another version is the one heard. None answering, none has
 * published.
 */
public final class PhotonVisionIOSim implements PhotonVisionIO {
  private final List<CoprocessorIOSim> coprocessors;
  private String last = "";

  public PhotonVisionIOSim(List<CoprocessorIOSim> coprocessors) {
    this.coprocessors = List.copyOf(coprocessors);
  }

  @Override
  public void updateInputs(PhotonVisionIOInputs inputs) {
    inputs.library = PhotonVersion.versionString;
    String now = "";
    for (CoprocessorIOSim coprocessor : coprocessors) {
      if (coprocessor.isAnswering()
          && (now.isEmpty() || !coprocessor.publishedVersion().equals(inputs.library))) {
        now = coprocessor.publishedVersion();
      }
    }
    // A version is published as a coprocessor's PhotonVision starts; the topic keeps the last.
    if (now.isEmpty()) {
      now = last;
    }
    inputs.published = now.equals(last) ? new String[] {} : new String[] {now};
    inputs.reported = now;
    last = now;
  }
}
