package frc.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Sensor.Switch.Limit.End;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.hardware.Hardware;
import frc.robot.hardware.RobotHardware;
import frc.robot.sim.SimulatedHardware;
import frc.robot.sim.SimulatedWorld;
import frc.robot.subsystems.mechanisms.AbsoluteEncoderIOPort;
import frc.robot.subsystems.mechanisms.JointIOSpark;
import frc.robot.subsystems.mechanisms.JointIOTalon;
import frc.robot.subsystems.mechanisms.PdhIORev;
import frc.robot.subsystems.mechanisms.SensorIO;
import frc.robot.subsystems.mechanisms.SensorIOCanRange;
import frc.robot.subsystems.mechanisms.SensorIOCandi;
import frc.robot.subsystems.mechanisms.SensorIOPort;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhotonVision;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.util.AlertDataJNI;

/**
 * The robot's hardware in each mode the program runs in: on a robot, the devices its definition's
 * wiring names, and nothing, said by an alert, for a part with none; in the simulator, the
 * simulated world's; in a replay, nothing, since the log has every reading.
 */
class HardwareTest {
  /** A pivot on a TalonFX, its encoder and lower limit switch on Systemcore ports. */
  private static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
          .range(Range.degrees(0, 135), Degrees.of(0))
          .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(11))
          .absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(2).offset(Rotations.of(0.3)))
          .limitSwitch(End.LOWER, Degrees.of(0), SensorWiring.port(1).normallyClosed())
          .control(MechanismControl.position(30).withKg(0));

  /** An arm on a SPARK, its lower limit switch on the SPARK's own input. */
  private static final Joint.Revolute ARM =
      Joint.pivot("arm", Part.CHASSIS, new Translation3d(-0.2, 0, 0.4), Joint.PITCH_UP)
          .range(Range.degrees(0, 90), Degrees.of(0))
          .motor(Motor.NEO, Gearing.of(50), SPARK_MAX.can(12))
          .limitSwitch(End.LOWER, Degrees.of(0), SensorWiring.Direction.REVERSE.normallyClosed())
          .control(
              MechanismControl.position(30)
                  .withKg(0)
                  .withHoming(MechanismControl.Homing.assumeAtReference()));

  /** A roller whose motor has no controller named: it runs in the simulator alone. */
  private static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))
          .outputs(Output.volts("intaking", 6))
          .motor(Motor.KRAKEN_X60, Gearing.of(3))
          .control(MechanismControl.velocity(0.02));

  private static final Sensor.Switch BEAM =
      Sensor.Switch.beam("claw beam", PIVOT).on(CANDI.can(30).s1().normallyOpen());
  private static final Sensor.Rangefinder HOPPER =
      Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
          .on(CANRANGE.can(31));

  /** A switch with no wiring. */
  private static final Sensor.Switch BUMPER = Sensor.Switch.contact("bumper", Part.CHASSIS);

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(Robots.SIM.definition())
          .name("hardware")
          .pdh(Pdh.can(1))
          .joints(List.of(PIVOT, ARM, ROLLER))
          .sensors(List.of(BEAM, HOPPER, BUMPER))
          .build();

  @RegisterExtension final RobotSim sim = new RobotSim();

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  /** Each of the robot's switches and rangefinders, its joints' limit switches among them. */
  private static Map<String, Sensor.Placed> sensors() {
    Map<String, Sensor.Placed> sensors = new LinkedHashMap<>();
    for (Sensor sensor : ROBOT.math().sensors()) {
      if (sensor instanceof Sensor.Placed placed) {
        sensors.put(placed.label(), placed);
      }
    }
    return sensors;
  }

  /**
   * On a robot, each part on the device its wiring names: the encoder and the limit switch on
   * Systemcore ports (each cast to its kind, to close it), a switch on a SPARK's input read with
   * its joint.
   */
  @Test
  void aRobotsHardwareIsTheDevicesItsWiringNames() {
    Map<String, Sensor.Placed> sensors = sensors();
    try (Hardware hardware = RobotContainer.hardware(Constants.Mode.REAL, ROBOT, null);
        AbsoluteEncoderIOPort encoder = (AbsoluteEncoderIOPort) hardware.absoluteEncoder(PIVOT);
        SensorIOPort lower =
            (SensorIOPort)
                hardware.sensor(Objects.requireNonNull(sensors.get("pivot lower limit")))) {
      assertThat(hardware).isInstanceOf(RobotHardware.class);
      assertThat(hardware.joint(PIVOT)).isInstanceOf(JointIOTalon.class);
      assertThat(hardware.joint(ARM)).isInstanceOf(JointIOSpark.class);
      assertThat(hardware.sensor(Objects.requireNonNull(sensors.get("arm lower limit"))))
          .isEqualTo(SensorIO.onPin("arm", SensorWiring.Direction.REVERSE));
      assertThat(hardware.sensor(BEAM)).isInstanceOf(SensorIOCandi.class);
      assertThat(hardware.sensor(HOPPER)).isInstanceOf(SensorIOCanRange.class);
      assertThat(hardware.pdh()).isInstanceOf(PdhIORev.class);
      List<CameraIO> cameras = hardware.cameras();
      assertThat(cameras)
          .hasSize(ROBOT.cameras().size())
          .allMatch(camera -> camera instanceof CameraIOPhotonVision);
      cameras.forEach(CameraIO::close);
    }
  }

  /**
   * On a robot, a part with no wiring gets nothing, and stays still rather than guess, and an alert
   * says which: the drivetrain, a joint, a sensor.
   */
  @Test
  void aRobotsPartWithNoWiringGetsNothingAndAnAlertSaysSo() {
    try (Hardware hardware = RobotContainer.hardware(Constants.Mode.REAL, ROBOT, null)) {
      assertThat(hardware.drive().modules())
          .allMatch(module -> module.getClass().isAnonymousClass());
      assertThat(hardware.joint(ROLLER).getClass().isAnonymousClass()).isTrue();
      assertThat(hardware.sensor(BUMPER).getClass().isAnonymousClass()).isTrue();

      assertThat(hardware.unwiredSensors()).containsExactly("bumper");
      assertThat(activeAlerts())
          .contains(
              "This robot's configuration has no drivetrain wiring (CAN IDs and encoder offsets),"
                  + " so the drivetrain stays still. Add it to the robot's entry in Robots.",
              "No wiring for roller, so they stay still. Add it to the robot's entry in Robots.",
              "No wiring for bumper, so nothing waiting on them holds. Add it to the robot's entry"
                  + " in Robots.");
    }
    assertThat(activeAlerts()).as("closed with it").noneMatch(text -> text.contains("No wiring"));
  }

  /** In the simulator, each part is the simulated world's, wired or not. */
  @Test
  void theSimulatorsHardwareIsTheWorlds() {
    try (SimulatedWorld world =
            new SimulatedWorld(
                RobotSim.FIELD,
                ROBOT.drivetrain(),
                ROBOT.body(),
                ROBOT.cameras(),
                ROBOT.imus(),
                SensorErrors.NONE);
        Hardware hardware = RobotContainer.hardware(Constants.Mode.SIM, ROBOT, world)) {
      assertThat(hardware).isInstanceOf(SimulatedHardware.class);
      assertThat(hardware.drive().modules())
          .containsExactlyElementsOf(world.drivetrainHardware().modules());
      hardware.joint(PIVOT);
      hardware.joint(ROLLER);
      hardware.absoluteEncoder(PIVOT);
      hardware.sensor(BUMPER);

      assertThat(Objects.requireNonNull(world.joints()).position(ROLLER))
          .as("the roller, unwired, simulated")
          .isZero();
      assertThat(world.sensors()).isNotNull();
      assertThat(hardware.pdh()).isSameAs(world.pdh());
      assertThat(hardware.cameras()).containsExactly(world.cameras());
      assertThat(hardware.unwiredSensors()).isEmpty();
    }
  }

  /** In a replay, nothing: every reading comes from the log. */
  @Test
  void aReplaysHardwareIsNothing() {
    try (Hardware hardware = RobotContainer.hardware(Constants.Mode.REPLAY, ROBOT, null)) {
      List<Object> made = new ArrayList<>(hardware.drive().modules());
      made.add(hardware.drive().gyro());
      made.add(hardware.joint(PIVOT));
      made.add(hardware.absoluteEncoder(PIVOT));
      made.add(hardware.sensor(BEAM));
      made.add(hardware.pdh());
      made.addAll(hardware.cameras());

      assertThat(made).allMatch(io -> io.getClass().isAnonymousClass());
      assertThat(hardware.cameras()).hasSize(ROBOT.cameras().size());
      assertThat(activeAlerts()).noneMatch(text -> text.contains("No wiring"));
    }
  }

  /** The alerts a dashboard shows now. */
  private static List<String> activeAlerts() {
    List<String> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0) {
        active.add(alert.text);
      }
    }
    return active;
  }
}
