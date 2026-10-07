package frc.robot.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.unmotored;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.RobotConfigBuilder;
import frc.robot.Robots;
import frc.robot.lib.wiring.ControllerPins;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Where a robot's switches and rangefinders are connected, each carrying its wiring: what a joint's
 * motor controller reads on its limits, and the IDs the definition holds them to.
 */
class SensorsTest {
  private static final CanBus BUS = CanBus.port(CanBus.Systemcore.CAN_S0);

  private static final Sensor.Rangefinder HOPPER_RANGE = range("hopper range");
  private static final Sensor.Rangefinder SHOOTER_RANGE = range("shooter range");

  /**
   * The definition holds a CANdi's and a CANrange's IDs to the rest of the bus's: a CANdi's two
   * inputs are one device, two CANranges on one ID aren't.
   */
  @Test
  void aCandisAndACanRangesIdsAreTheBussOwn() {
    Sensor.Switch down =
        Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)
            .on(CANDI.can(BUS, 31).s1().normallyClosed());
    Sensor.Switch up =
        Sensor.Switch.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER)
            .on(CANDI.can(BUS, 31).s2().normallyClosed());
    RobotDefinition wired =
        RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
            .sensors(List.of(down, up, HOPPER_RANGE.on(CANRANGE.can(BUS, 30))))
            .build();

    assertThat(RobotConfigBuilder.builder(Robots.SIM).definition(wired).build().definition())
        .isEqualTo(wired);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(wired)
                    .sensors(
                        List.of(
                            down,
                            up,
                            HOPPER_RANGE.on(CANRANGE.can(BUS, 30)),
                            SHOOTER_RANGE.on(CANRANGE.can(BUS, 30))))
                    .build())
        .withMessage(
            "RobotDefinition[joints].devices: two CANranges on Systemcore's CAN_S0 port share CAN"
                + " ID 30, hopper range and shooter range: give each its own");
  }

  /**
   * A joint's controller's limits come from the wiring its switches carry: its own limit switch on
   * its pin, stopped on; a beam on its other pin, read only; and on a TalonFX, its limit switch on
   * a CANdi on its bus, read over CAN as the limit for the switch's end. A SPARK can't read a
   * CANdi, and a joint with nothing on it has nothing.
   */
  @Test
  void eachJointsControllerLimitsComeFromTheWiring() {
    Joint.Revolute onSpark =
        unmotored(PIVOT).motor(Motor.NEO, JointFixtures.gearing(25), SPARK_MAX.can(21));
    Joint.Revolute onTalon =
        unmotored(PIVOT).motor(Motor.FALCON_500, JointFixtures.gearing(25), TALON_FX.can(21));
    Joint.Revolute roller = JointFixtures.ROLLER.motorsOn(TALON_FX.can(22));
    Sensor.Switch up =
        Sensor.Switch.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER)
            .on(CANDI.can(BUS, 31).s2().normallyClosed());
    List<Sensor> wired =
        List.of(
            Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)
                .on(
                    SensorWiring.ControllerLimit.of(PIVOT, SensorWiring.Direction.REVERSE)
                        .normallyClosed()),
            Sensor.Switch.beam("claw beam", PIVOT)
                .on(
                    SensorWiring.ControllerLimit.of(PIVOT, SensorWiring.Direction.FORWARD)
                        .normallyOpen()),
            up);
    RobotDefinition onSparks = robot(List.of(onSpark, roller), wired);
    RobotDefinition onTalons = robot(List.of(onTalon, roller), wired);

    ControllerPins spark = Sensors.pins(onSparks, onSpark);
    ControllerPins talon = Sensors.pins(onTalons, onTalon);

    assertThat(spark.reverse())
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    assertThat(spark.forward())
        .as("a beam, and a SPARK can't read the CANdi")
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false));
    assertThat(talon.forward())
        .as(
            "its pin's beam: the CANdi's upper switch would be its forward too, and the pin keeps it")
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false));
    assertThat(Sensors.pins(robot(List.of(onTalon, roller), List.of(up)), onTalon).forward())
        .contains(new ControllerPins.OnCandi(31, 2));
    assertThat(Sensors.pins(onSparks, roller)).isEqualTo(ControllerPins.NONE);
  }

  /** The fixtures' robot with its pivot and roller as given, and these sensors. */
  private static RobotDefinition robot(List<Joint> pivotAndRoller, List<Sensor> sensors) {
    return RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
        .joints(
            List.of(
                pivotAndRoller.get(0),
                pivotAndRoller.get(1),
                JointFixtures.ELEVATOR,
                JointFixtures.TURRET))
        .sensors(sensors)
        .build();
  }

  private static Sensor.Rangefinder range(String label) {
    return Sensor.Rangefinder.builder()
        .label(label)
        .part(Part.CHASSIS)
        .range(Meters.of(1.3))
        .fieldOfView(Degrees.of(27))
        .build();
  }
}
