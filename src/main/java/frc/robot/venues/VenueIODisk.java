package frc.robot.venues;

import java.io.IOException;
import java.util.Optional;

/**
 * The selected venue on the robot's disk: read at the first loop, and again after the pit selects
 * another. Never read every loop: a venue changes once an event.
 */
public final class VenueIODisk implements VenueIO {
  private final VenueStore store;
  private boolean read;

  public VenueIODisk(VenueStore store) {
    this.store = store;
  }

  /** Reads the disk again at the next loop: after a selection. */
  public void reread() {
    read = false;
  }

  @Override
  public void updateInputs(VenueIOInputs inputs) {
    if (read) {
      return;
    }
    read = true;
    try {
      Optional<String> text = store.selectedText();
      inputs.text = text.orElse("");
      inputs.problem = "";
      inputs.inGit = text.map(store::committed).orElse(true);
    } catch (IOException | RuntimeException e) {
      inputs.text = "";
      inputs.problem = String.valueOf(e.getMessage());
      inputs.inGit = true;
    }
  }
}
