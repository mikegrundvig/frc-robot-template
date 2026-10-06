package frc.robot.lib.wiring;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * Every CAN device's ID, across the drivetrain and the mechanisms. Two devices of one kind on one
 * bus can't share an ID: each would answer as the other, and two parts of the program would drive
 * one motor. Devices of different kinds may: a TalonFX and a SPARK, or a SPARK MAX and a CANcoder.
 * A SPARK MAX and a SPARK Flex are one kind, REV's SPARK.
 */
public final class CanIds {
  private CanIds() {}

  /** The modules' names, in the wiring's order. */
  private static final List<String> MODULES =
      List.of("front left", "front right", "back left", "back right");

  private static final String ENCODER = "CANcoder";

  /** Where a device is, and what kind it is: what two devices may not both be. */
  private record Place(String kind, CanBus bus, int id) {}

  /**
   * What's wrong: each ID two devices of one kind share on one bus, naming both. Null when none do.
   *
   * @param drivetrain the drivetrain's wiring, if it has any
   * @param mechanisms each mechanism's wiring, by its joint's label
   */
  public static @Nullable String twice(
      Optional<SwerveWiring> drivetrain, Map<String, MechanismWiring> mechanisms) {
    List<String> problems = clashes(drivetrain, mechanisms, List.of());
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /**
   * The same for the sensors on CAN, a CANrange or a CANdi (whose two inputs are one device): each
   * ID one shares with another device of its kind. Null when none do; what the drivetrain and the
   * mechanisms share among themselves is {@link #twice}'s to say.
   *
   * @param sensors each sensor's wiring
   */
  public static @Nullable String sensorsTwice(
      Optional<SwerveWiring> drivetrain,
      Map<String, MechanismWiring> mechanisms,
      List<SensorWiring> sensors) {
    List<String> problems = clashes(drivetrain, mechanisms, sensors);
    problems.removeAll(clashes(drivetrain, mechanisms, List.of()));
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  private static List<String> clashes(
      Optional<SwerveWiring> drivetrain,
      Map<String, MechanismWiring> mechanisms,
      List<SensorWiring> sensors) {
    Map<Place, String> owners = new LinkedHashMap<>();
    List<String> problems = new ArrayList<>();
    drivetrain.ifPresent(
        wiring -> {
          add(owners, problems, "Pigeon 2", wiring.canBus(), wiring.gyroId(), "the gyro");
          for (int i = 0; i < wiring.modules().size(); i++) {
            SwerveWiring.ModuleWiring module = wiring.modules().get(i);
            String name = "the " + MODULES.get(i) + " module's";
            add(
                owners,
                problems,
                wiring.driveController().kind(),
                wiring.canBus(),
                module.driveId(),
                name + " drive");
            add(
                owners,
                problems,
                wiring.steerController().kind(),
                wiring.canBus(),
                module.steerId(),
                name + " steering");
            module
                .encoderId()
                .ifPresent(
                    id -> add(owners, problems, ENCODER, wiring.canBus(), id, name + " CANcoder"));
          }
        });
    mechanisms.forEach(
        (label, wiring) -> {
          for (MechanismWiring.Motor motor : wiring.motors()) {
            add(owners, problems, wiring.controller().kind(), wiring.canBus(), motor.id(), label);
          }
          wiring
              .cancoder()
              .ifPresent(
                  cancoder ->
                      add(
                          owners,
                          problems,
                          ENCODER,
                          wiring.canBus(),
                          cancoder.id(),
                          label + "'s CANcoder"));
        });
    for (SensorWiring sensor : sensors) {
      switch (sensor.input()) {
        case SensorWiring.CanRange range ->
            add(owners, problems, "CANrange", range.bus(), range.id(), sensor.sensor());
        case SensorWiring.Candi candi ->
            add(owners, problems, "CANdi", candi.bus(), candi.id(), "CANdi " + candi.id());
        case SensorWiring.Port port -> {}
        case SensorWiring.ControllerLimit limit -> {}
      }
    }
    return problems;
  }

  private static void add(
      Map<Place, String> owners,
      List<String> problems,
      String kind,
      CanBus bus,
      int id,
      String owner) {
    String other = owners.putIfAbsent(new Place(kind, bus, id), owner);
    if (other != null && !other.equals(owner)) {
      problems.add(
          "two "
              + kind
              + "s on "
              + shown(bus)
              + " share CAN ID "
              + id
              + ", "
              + other
              + " and "
              + owner
              + ": give each its own");
    }
  }

  private static String shown(CanBus bus) {
    return switch (bus) {
      case CanBus.Port port -> "Systemcore's " + port.port() + " port";
      case CanBus.Canivore canivore -> "the CANivore " + canivore.name();
    };
  }
}
