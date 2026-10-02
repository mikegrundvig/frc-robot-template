package frc.robot.subsystems.coprocessors;

import java.util.Arrays;
import org.photonvision.PhotonVersion;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.StringSubscriber;

/**
 * PhotonVision's version as it publishes it on the robot's NetworkTables ({@code
 * /photonvision/version}), and PhotonLib's ({@code PhotonVersion.versionString}).
 */
public final class PhotonVisionIONetworkTables implements PhotonVisionIO {
  /** Where PhotonVision publishes its version, as PhotonLib reads it. */
  static final String TOPIC = "/photonvision/version";

  private final StringSubscriber version;

  public PhotonVisionIONetworkTables(NetworkTableInstance networkTables) {
    // Every value kept between loops, so a version a coprocessor published in passing is seen.
    version = networkTables.getStringTopic(TOPIC).subscribe("", PubSubOption.pollStorage(10));
  }

  @Override
  public void updateInputs(PhotonVisionIOInputs inputs) {
    inputs.library = PhotonVersion.versionString;
    inputs.published =
        Arrays.stream(version.readQueue()).map(each -> each.value).toArray(String[]::new);
    inputs.reported = version.get();
  }

  @Override
  public void close() {
    version.close();
  }
}
