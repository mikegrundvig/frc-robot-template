package frc.robot.subsystems.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.photonvision.PhotonVersion;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;

/** PhotonVision's version as it publishes it, read from NetworkTables as on a robot. */
class PhotonVisionIONetworkTablesTest {
  @Test
  void readsWhatPhotonVisionPublishesBesidePhotonLibs() {
    try (NetworkTableInstance networkTables = NetworkTableInstance.create();
        PhotonVisionIONetworkTables io = new PhotonVisionIONetworkTables(networkTables);
        StringPublisher coprocessor =
            networkTables.getStringTopic(PhotonVisionIONetworkTables.TOPIC).publish()) {
      PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();

      io.updateInputs(inputs);
      String before = inputs.reported;
      coprocessor.set("v2026.3.4");
      coprocessor.set("v2027.1.0");
      io.updateInputs(inputs);

      assertThat(before).isEmpty();
      assertThat(inputs.library).isEqualTo(PhotonVersion.versionString);
      assertThat(inputs.reported).isEqualTo("v2027.1.0");
      assertThat(inputs.published).containsExactly("v2026.3.4", "v2027.1.0");
    }
  }
}
