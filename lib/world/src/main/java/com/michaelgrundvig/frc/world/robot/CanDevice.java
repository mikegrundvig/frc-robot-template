package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * One device on one of the robot's CAN buses, as the robot's definition wires it: what it is, its
 * bus and ID, and the PDH channel that feeds it, where its wiring says. {@code
 * RobotDefinition.devices()} lists every one, once, worked out from where each is written: the
 * drivetrain's wiring, each joint's block, and the wiring its sensors carry. Nothing is written
 * twice to make it, and the robot's checks across its wiring read it: no two of a kind on one bus
 * at one ID ({@link #sharedIds}), no PDH channel feeding two.
 *
 * @param name what the robot's messages and logs call it: "pivot motor 32", "front-left drive",
 *     "pivot CANcoder", "hopper range"
 * @param kind what it is: two of a kind can't share an ID on one bus
 * @param bus the CAN bus it's on
 * @param id its CAN ID
 * @param feed the PDH channel it's powered from, and its breaker, where its wiring says: only a
 *     motor controller's says, so far
 */
public record CanDevice(String name, Kind kind, CanBus bus, int id, Optional<Pdh.Feed> feed) {
  /** What a device on the CAN bus is: the kinds that can't share an ID with their own. */
  public enum Kind {
    TALON_FX("TalonFX", true),
    TALON_FXS("TalonFXS", true),
    /** REV's SPARK MAX and SPARK Flex: one kind on the bus. */
    SPARK("SPARK", true),
    CANCODER("CANcoder", false),
    PIGEON_2("Pigeon 2", false),
    CANDI("CANdi", false),
    CANRANGE("CANrange", false),
    PDH("PDH", false);

    private final String shown;
    private final boolean motorController;

    Kind(String shown, boolean motorController) {
      this.shown = shown;
      this.motorController = motorController;
    }

    /** Whether it's a motor controller, which the PDH feeds from a channel of its own. */
    public boolean motorController() {
      return motorController;
    }

    /** As a sentence names it: "TalonFX", "Pigeon 2". */
    @Override
    public String toString() {
      return shown;
    }
  }

  /**
   * Each ID two devices of one kind share on one bus, naming both, about the second: each would
   * answer as the other, and two parts of the program would drive one motor. Devices of different
   * kinds may share one: a TalonFX and a SPARK, or a SPARK and a CANcoder.
   *
   * @param devices every device on the robot's buses ({@code RobotDefinition.devices})
   */
  static List<Finding> sharedIds(List<CanDevice> devices) {
    record Place(Kind kind, CanBus bus, int id) {}
    Map<Place, String> owners = new HashMap<>();
    List<Finding> shared = new ArrayList<>();
    for (CanDevice device : devices) {
      String other =
          owners.putIfAbsent(new Place(device.kind(), device.bus(), device.id()), device.name());
      if (other != null) {
        shared.add(
            Finding.refused(
                device.name(),
                "two "
                    + device.kind()
                    + "s on "
                    + device.bus().shown()
                    + " share CAN ID "
                    + device.id()
                    + ", "
                    + other
                    + " and "
                    + device.name()
                    + ": give each its own"));
      }
    }
    return shared;
  }
}
