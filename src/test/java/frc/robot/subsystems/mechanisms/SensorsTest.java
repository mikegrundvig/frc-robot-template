package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.RobotConfigBuilder;
import frc.robot.Robots;
import frc.robot.lib.wiring.CanBus;
import frc.robot.lib.wiring.ControllerPins;
import frc.robot.lib.wiring.MechanismWiring;
import frc.robot.lib.wiring.SensorWiring;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

/**
 * Where a robot's switches and rangefinders are connected, checked against what its definition says
 * they are and against its joints' wiring, as its configuration is built.
 */
class SensorsTest {
  private static final CanBus BUS = CanBus.port(CANPort.CAN_S0);

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .sensors(
              List.of(
                  Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER),
                  Sensor.Switch.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER),
                  Sensor.Switch.beam("claw beam", PIVOT),
                  Sensor.Switch.contact("cage", Part.CHASSIS),
                  Sensor.Rangefinder.builder()
                      .label("hopper range")
                      .part(Part.CHASSIS)
                      .pose(Optional.empty())
                      .range(Meters.of(1.3))
                      .fieldOfView(Degrees.of(27))
                      .build(),
                  Sensor.Rangefinder.builder()
                      .label("shooter range")
                      .part(Part.CHASSIS)
                      .pose(Optional.empty())
                      .range(Meters.of(1.3))
                      .fieldOfView(Degrees.of(27))
                      .build()))
          .build();

  /** The pivot on a SPARK and the roller on a TalonFX; the others unwired. */
  private static final List<JointSetup> JOINTS =
      List.of(
          wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 21),
          wired(SETUPS.get(1), MechanismWiring.Controller.TALON_FX, 22),
          SETUPS.get(2),
          SETUPS.get(3));

  /**
   * Each switch on one of the inputs a switch can be on (a SPARK's limit inputs, a joint's own
   * limit switch on one and a beam on the other; a CANdi's; a Systemcore port), either polarity,
   * and the rangefinders on CANranges: nothing to say.
   */
  @Test
  void aSwitchGoesOnAnySwitchInputAndARangefinderOnACanRange() {
    assertThat(Sensors.wiringProblems(ROBOT, wired(JOINTS), everySensor())).isNull();
  }

  /**
   * A wiring for a sensor the robot doesn't have, a switch wired as a rangefinder, a sensor wired
   * twice, two on one port, a controller's limit input on a Kraken's TalonFX, which has none, or on
   * a joint that isn't wired, and a joint's limit switch on another's: each named. A beam on a
   * SPARK's limit input is fine.
   */
  @Test
  void itNamesEachSensorItCantRead() {
    List<SensorWiring> wrong =
        List.of(
            onPort("stray", 0),
            SensorWiring.OfRangefinder.builder()
                .sensor("cage")
                .input(new SensorWiring.CanRange(BUS, 40))
                .build(),
            onPort("claw beam", 2),
            onPort("claw beam", 3),
            onCandi("pivot down", 1),
            onCandi("pivot up", 1),
            SensorWiring.OfSwitch.builder()
                .sensor("pivot down")
                .input(new SensorWiring.ControllerLimit("roller", SensorWiring.Direction.REVERSE))
                .polarity(SensorWiring.Polarity.NORMALLY_CLOSED)
                .build(),
            SensorWiring.OfSwitch.builder()
                .sensor("pivot up")
                .input(new SensorWiring.ControllerLimit("elevator", SensorWiring.Direction.FORWARD))
                .polarity(SensorWiring.Polarity.NORMALLY_CLOSED)
                .build());

    assertThat(Sensors.wiringProblems(ROBOT, wired(JOINTS), wrong))
        .isEqualTo(
            "stray is wired, and isn't one of joints's sensors; cage is wired as a rangefinder,"
                + " and the definition's isn't one; claw beam is wired twice; pivot down and"
                + " pivot up are on one input: CANdi 31's S1; pivot down is wired twice; pivot down"
                + " is pivot's limit switch, on roller's controller, which would stop roller: wire"
                + " it to pivot's; pivot down is on roller's TalonFX's limit input, and a Kraken's"
                + " TalonFX has none: put it on a CANdi; pivot up is wired twice; pivot up is"
                + " pivot's limit switch, on elevator's controller, which would stop elevator: wire"
                + " it to pivot's; pivot up is on elevator's controller's limit input, and elevator"
                + " isn't wired");
  }

  /**
   * What an input can't be is refused as its wiring is built: a switch on a CANrange, a rangefinder
   * on a CANdi, a seventh Systemcore port.
   */
  @Test
  void aWiringRefusesAnInputItsSensorCantBeOn() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                SensorWiring.OfSwitch.builder()
                    .sensor("claw beam")
                    .input(new SensorWiring.CanRange(BUS, 30))
                    .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
                    .build())
        .withMessageContaining("a switch isn't on a CANrange: a CANrange is a rangefinder");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                SensorWiring.OfRangefinder.builder()
                    .sensor("hopper range")
                    .input(new SensorWiring.Candi(BUS, 30, 1))
                    .build())
        .withMessageContaining("a rangefinder is on a CANrange");
    assertThatIllegalArgumentException().isThrownBy(() -> port(6));
  }

  /**
   * The configuration asks the same as it's built, and holds a CANdi's and a CANrange's IDs to the
   * rest of the bus's: a CANdi's two inputs are one device, two CANranges on one ID aren't.
   */
  @Test
  void theConfigurationRefusesSensorsItCantRead() {
    List<JointSetup> joints = List.of(SETUPS.get(0), JOINTS.get(1), SETUPS.get(2), SETUPS.get(3));
    List<SensorWiring> onCan =
        List.of(
            onCandi("pivot down", 1),
            onCandi("pivot up", 2),
            SensorWiring.OfRangefinder.builder()
                .sensor("hopper range")
                .input(new SensorWiring.CanRange(BUS, 30))
                .build());
    var robot =
        RobotConfigBuilder.builder(Robots.SIM).definition(ROBOT).joints(joints).sensors(onCan);
    assertThat(robot.build().sensors()).hasSize(3);

    List<SensorWiring> shared = new ArrayList<>(onCan);
    shared.add(
        SensorWiring.OfRangefinder.builder()
            .sensor("shooter range")
            .input(new SensorWiring.CanRange(BUS, 30))
            .build());
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot.sensors(shared).build())
        .withMessage(
            "RobotConfig[joints].sensors: two CANranges on Systemcore's CAN_S0 port share CAN ID"
                + " 30, hopper range and shooter range: give each its own");
  }

  /**
   * A joint's controller's limits come from the wiring: its own limit switch on its pin, stopped
   * on; a beam on its other pin, read only; and on a TalonFX, its limit switch on a CANdi on its
   * bus, read over CAN as the limit for the switch's end. A SPARK can't read a CANdi, and a joint
   * with nothing on it has nothing.
   */
  @Test
  void eachJointsControllerLimitsComeFromTheWiring() {
    List<SensorWiring> wiring =
        List.of(
            SensorWiring.OfSwitch.builder()
                .sensor("pivot down")
                .input(new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.REVERSE))
                .polarity(SensorWiring.Polarity.NORMALLY_CLOSED)
                .build(),
            SensorWiring.OfSwitch.builder()
                .sensor("claw beam")
                .input(new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.FORWARD))
                .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
                .build(),
            onCandi("pivot up", 2));
    JointSetup onTalon = wired(SETUPS.get(0), MechanismWiring.Controller.TALON_FX, 21);

    ControllerPins spark = Sensors.pins(ROBOT, wiring, JOINTS.get(0));
    ControllerPins talon = Sensors.pins(ROBOT, wiring, onTalon);

    assertThat(spark.reverse())
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    assertThat(spark.forward())
        .as("a beam, and a SPARK can't read the CANdi")
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false));
    assertThat(talon.forward())
        .as(
            "its pin's beam: the CANdi's upper switch would be its forward too, and the pin keeps it")
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false));
    assertThat(Sensors.pins(ROBOT, List.of(onCandi("pivot up", 2)), onTalon).forward())
        .contains(new ControllerPins.OnCandi(31, 2));
    assertThat(Sensors.pins(ROBOT, wiring, JOINTS.get(1))).isEqualTo(ControllerPins.NONE);
  }

  private static List<SensorWiring> everySensor() {
    return List.of(
        SensorWiring.OfSwitch.builder()
            .sensor("pivot down")
            .input(new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.REVERSE))
            .polarity(SensorWiring.Polarity.NORMALLY_CLOSED)
            .build(),
        onCandi("pivot up", 1),
        SensorWiring.OfSwitch.builder()
            .sensor("claw beam")
            .input(new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.FORWARD))
            .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
            .build(),
        onPort("cage", 4),
        SensorWiring.OfRangefinder.builder()
            .sensor("hopper range")
            .input(new SensorWiring.CanRange(BUS, 30))
            .build());
  }

  private static SensorWiring onPort(String sensor, int port) {
    return SensorWiring.OfSwitch.builder()
        .sensor(sensor)
        .input(port(port))
        .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
        .build();
  }

  private static SensorWiring onCandi(String sensor, int input) {
    return SensorWiring.OfSwitch.builder()
        .sensor(sensor)
        .input(new SensorWiring.Candi(BUS, 31, input))
        .polarity(SensorWiring.Polarity.NORMALLY_CLOSED)
        .build();
  }

  private static SensorWiring.Port port(int port) {
    return new SensorWiring.Port(port);
  }

  private static Map<String, MechanismWiring> wired(List<JointSetup> joints) {
    Map<String, MechanismWiring> wired = new HashMap<>();
    for (JointSetup setup : joints) {
      setup.wiring().ifPresent(wiring -> wired.put(setup.joint().label(), wiring));
    }
    return wired;
  }

  private static JointSetup wired(JointSetup setup, MechanismWiring.Controller controller, int id) {
    return JointSetup.builder()
        .joint(setup.joint())
        .control(setup.control())
        .wiring(
            Optional.of(
                MechanismWiring.builder()
                    .canBus(BUS)
                    .controller(controller)
                    .motors(List.of(MechanismWiring.Motor.builder().id(id).clockwise(true).build()))
                    .cancoder(Optional.empty())
                    .dataPort(Optional.empty())
                    .build()))
        .build();
  }
}
