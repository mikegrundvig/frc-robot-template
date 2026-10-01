package frc.robot.subsystems.coprocessors;

import org.littletonrobotics.junction.AutoLog;

/**
 * Which PhotonVision the coprocessors run, as PhotonVision itself says, against the PhotonLib this
 * robot program was built with. PhotonLib's own check compares only their message formats, so two
 * versions that share one pass it silently; this compares the versions themselves, as exact
 * strings.
 */
public interface PhotonVisionIO extends AutoCloseable {
  @AutoLog
  class PhotonVisionIOInputs {
    /** The PhotonLib this program was built with: its version string. */
    public String library = "";

    /**
     * The version PhotonVision publishes ({@code /photonvision/version}), empty until one does.
     * Every coprocessor publishes to that one topic, so it holds whichever spoke last.
     */
    public String reported = "";

    /** Every version published since the last loop, in order: one per coprocessor starting. */
    public String[] published = new String[] {};
  }

  default void updateInputs(PhotonVisionIOInputs inputs) {}

  @Override
  default void close() {}
}
