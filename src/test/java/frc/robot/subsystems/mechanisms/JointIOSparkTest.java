package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROLLER;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET;
import static frc.robot.subsystems.mechanisms.JointFixtures.gearing;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.MechanismWiringBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;
import frc.robot.RobotSim;
import frc.robot.lib.rev.RevMechanism;
import frc.robot.lib.wiring.ControllerPins;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.system.DCMotor;

/**
 * The REV joint against REV's own simulated SPARK: what it tells the SPARK, in the SPARK's units,
 * and what it reads back, in the joint's. The settings are the REV helper's, from the Phoenix
 * helper's, tested there.
 */
@ExtendWith(RobotSim.class)
class JointIOSparkTest {
  /** The fixtures' robot on REV's motors: a NEO pivot through 25:1, two Vortexes on the slide. */
  static final RobotDefinition ON_REV =
      RobotDefinitionBuilder.builder(ROBOT)
          .drives(
              List.of(
                  JointFixtures.motors(PIVOT, Motor.NEO, 1, gearing(25)),
                  JointFixtures.motors(ROLLER, Motor.NEO_550, 1, gearing(3)),
                  JointFixtures.motors(ELEVATOR, Motor.NEO_VORTEX, 2, gearing(10), Inches.of(1)),
                  JointFixtures.motors(TURRET, Motor.NEO, 1, gearing(50))))
          .build();

  private static JointSetup wired(
      JointSetup setup, MechanismWiring.Controller controller, int... ids) {
    return JointSetupBuilder.builder(setup)
        .wiring(
            Optional.of(
                MechanismWiring.builder()
                    .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
                    .controller(controller)
                    .motors(
                        java.util.Arrays.stream(ids)
                            .mapToObj(
                                id ->
                                    MechanismWiring.Motor.builder().id(id).clockwise(false).build())
                            .toList())
                    .cancoder(Optional.empty())
                    .dataPort(Optional.empty())
                    .build()))
        .build();
  }

  /**
   * A pivot read by its rotor starts where its definition rests it, and reads its rotor's turns
   * through its 25:1: twenty-five more are one more turn of the pivot.
   */
  @Test
  void itReadsItsRotorThroughItsGearingFromWhereItRests() {
    JointIOSpark io =
        new JointIOSpark(
            ON_REV.math(), wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 31));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double resting = inputs.position;
    // The simulation's own position starts at 0, not where the startup set the encoder.
    sim.setPosition(PIVOT.reference() / (2 * Math.PI) * 25 + 25);
    sim.iterate(0, 12, 0.02);
    io.updateInputs(inputs);

    assertThat(resting)
        .as("it starts at its reference, 1.5°")
        .isCloseTo(PIVOT.reference(), within(1e-9));
    // A SPARK reports its position as a float.
    assertThat(inputs.position).isCloseTo(PIVOT.reference() + 2 * Math.PI, within(1e-6));
    assertThat(inputs.unconfigured).as("every SPARK took its settings").isEmpty();
  }

  /**
   * The SPARK is told each limit input's polarity, and stops on a limit switch only: a beam read
   * through its forward input lets the motor run on. REV applies the polarity on the device (its
   * docs: {@code isPressed} is "based on the selected polarity"), and its simulator's "pressed" is
   * after it, so what the SPARK took is read back from it, and a press reads tripped.
   *
   * <p>REVLib 2027.0.0-alpha-8 reads a limit input's type back the other way round: set normally
   * open, it reads normally closed, and a fresh SPARK reads normally closed where REV's docs say
   * its default is normally open, which points at the reading, not the setting. Pinned here, so a
   * REVLib that changes it fails this test; which side is right waits on a SPARK on the bench (open
   * question M22).
   */
  @Test
  void itsLimitInputsTakeTheirPolarityAndStopOnlyOnALimitSwitch() {
    ControllerPins pins =
        ControllerPins.NONE
            .with(
                SensorWiring.Direction.FORWARD,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false))
            .with(
                SensorWiring.Direction.REVERSE,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    JointIOSpark io =
        new JointIOSpark(
            ON_REV.math(), wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 47), pins);
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    LimitSwitchConfigAccessor took = ((SparkMax) io.leader()).configAccessor.limitSwitch;

    sim.getForwardLimitSwitchSim().setPressed(true);
    io.updateInputs(inputs);

    assertThat(took.getForwardSwitchType())
        .as("set normally open; alpha-8 reads it back the other way")
        .isEqualTo(LimitSwitchConfig.Type.kNormallyClosed);
    assertThat(took.getReverseSwitchType())
        .as("set normally closed; alpha-8 reads it back the other way")
        .isEqualTo(LimitSwitchConfig.Type.kNormallyOpen);
    assertThat(took.getForwardLimitSwitchTriggerBehavior())
        .as("a beam: read, never stopped on")
        .isEqualTo(LimitSwitchConfig.Behavior.kKeepMovingMotor);
    assertThat(took.getReverseLimitSwitchTriggerBehavior())
        .as("a limit switch: the SPARK stops on it")
        .isEqualTo(LimitSwitchConfig.Behavior.kStopMovingMotor);
    assertThat(inputs.forwardLimit).isTrue();
    assertThat(inputs.reverseLimit).isFalse();
  }

  /**
   * A position goes to the SPARK's loop in rotor turns, with the feedforward the robot works out: a
   * pivot 45° up is 25 × 1/8 rotor turns. A little past where it rests, its loop's output is its kP
   * on the error plus the feedforward's share of 12 V (far off, the current limit caps it).
   */
  @Test
  void aPositionGoesInRotorTurnsWithItsFeedforwardInVolts() {
    JointSetup setup = wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 32);
    JointIOSpark io = new JointIOSpark(ON_REV.math(), setup);
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    sim.enable();
    RevMechanism.Settings settings =
        RevMechanism.configure(
            ON_REV.math(),
            PIVOT,
            ON_REV.math().drivesOf(PIVOT).get(0),
            setup.wiring().orElseThrow(),
            setup.control());

    // REV's simulation steps its own position into the encoder: start it where the pivot rests.
    sim.setPosition(PIVOT.reference() / (2 * Math.PI) * 25);
    io.setPosition(Math.toRadians(45), 0, 0.6);
    sim.iterate(0, 12, 0.02);
    double far = sim.getSetpoint();
    double near = PIVOT.reference() + Math.toRadians(0.5);
    io.setPosition(near, 0, 0.6);
    for (int i = 0; i < 25; i++) {
      sim.iterate(0, 12, 0.02);
    }

    double rest = PIVOT.reference() / (2 * Math.PI) * 25;
    double target = near / (2 * Math.PI) * 25;
    double feedforward = 0.6 + settings.feedforward().atPosition(near / (2 * Math.PI), 0, 0);
    // A SPARK holds its setpoint as a float.
    assertThat(far).isCloseTo(25.0 / 8, within(1e-6));
    assertThat(sim.getSetpoint()).isCloseTo(target, within(1e-6));
    assertThat(sim.getAppliedOutput())
        .isCloseTo(settings.positionKp() * (target - rest) + feedforward / 12, within(1e-3));
  }

  /** The pit's tuning reaches the SPARK's loop, as a TalonFX's slot: its kP, in rotor turns. */
  @Test
  void aTunedGainReachesItsLoop() {
    JointSetup setup = wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 33);
    JointIOSpark io = new JointIOSpark(ON_REV.math(), setup);
    MechanismControl tuned =
        MechanismControlBuilder.builder(setup.control())
            .gains(MechanismControlGainsBuilder.builder(setup.control().gains()).kP(60).build())
            .build();

    io.retune(tuned);

    // 60 V a radian is 60 × 2π a turn of the pivot, 25 rotor turns, as a share of 12 V.
    double expected = 60 * 2 * Math.PI / (12 * 25);
    assertThat(((SparkMax) io.leader()).configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(expected, within(1e-6 * expected));
  }

  /**
   * The slide's two Vortexes on SPARK Flexes: the first leads, and reads meters through its drum.
   */
  @Test
  void aSlideOnTwoSparkFlexesReadsMetersThroughItsDrum() {
    JointIOSpark io =
        new JointIOSpark(
            ON_REV.math(), wired(SETUPS.get(2), MechanismWiring.Controller.SPARK_FLEX, 34, 35));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNeoVortex(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double start = inputs.position;
    // Ten rotor turns through 10:1 are one turn of the 1 in drum: 2π inches of travel.
    sim.setPosition(sim.getPosition() + 10);
    sim.iterate(0, 12, 0.02);
    io.updateInputs(inputs);

    assertThat(inputs.position - start).isCloseTo(2 * Math.PI * 0.0254, within(1e-9));
    assertThat(inputs.unconfigured).isEmpty();
  }

  /** A speed goes to the SPARK's speed slot in rotor RPM: the roller's 1 rad/s through 3:1. */
  @Test
  void aSpeedGoesInRotorRpm() {
    JointIOSpark io =
        new JointIOSpark(
            ON_REV.math(), wired(SETUPS.get(1), MechanismWiring.Controller.SPARK_MAX, 36));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNeo550(1));
    sim.enable();

    io.setVelocity(1);
    sim.iterate(0, 12, 0.02);

    assertThat(sim.getSetpoint()).isCloseTo(1 / (2 * Math.PI) * 3 * 60, within(1e-4));
    assertThat(sim.getClosedLoopSlot()).isEqualTo(ClosedLoopSlot.kSlot1);
  }

  /**
   * Homing pushes past the soft limits, which a SPARK keeps itself: they're switched off while it
   * homes, and back on once it's found where it is.
   */
  @Test
  void homingSwitchesItsSoftLimitsOffAndBackOn() {
    JointSetup setup =
        JointSetupBuilder.builder(wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 37))
            .control(SETUPS.get(0).control().withSoftLimits(0.01, 2.3))
            .build();
    JointIOSpark io = new JointIOSpark(ON_REV.math(), setup);
    SparkMax spark = (SparkMax) io.leader();
    boolean before = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();

    io.home(-2);
    boolean homing = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();
    io.resetPosition(0);
    boolean after = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();

    assertThat(before).isTrue();
    assertThat(homing).isFalse();
    assertThat(after).isTrue();
  }

  /**
   * A pivot with REV's Through Bore on its SPARK's data port: powered on at 60°, not where it
   * rests, it's told so by the encoder's first good reading, and its motor's encoder carries it on.
   */
  @Test
  void aPivotWithAnEncoderOnItsDataPortIsToldWhereItIs() {
    RobotDefinition withEncoder =
        RobotDefinitionBuilder.builder(ON_REV)
            .sensors(List.of(Sensor.Encoder.absolute("pivot absolute", PIVOT, 8192)))
            .build();
    JointSetup setup =
        JointSetupBuilder.builder(wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 38))
            .wiring(
                Optional.of(
                    MechanismWiringBuilder.builder(
                            wired(SETUPS.get(0), MechanismWiring.Controller.SPARK_MAX, 38)
                                .wiring()
                                .orElseThrow())
                        .dataPort(
                            Optional.of(
                                MechanismWiring.DataPortEncoder.builder()
                                    .offset(Rotations.of(-0.1))
                                    .reversed(false)
                                    .build()))
                        .build()))
            .build();
    JointIOSpark io = new JointIOSpark(withEncoder.math(), setup);
    // Its raw reading at 60° is 60° of a turn, less the offset.
    new SparkAbsoluteEncoderSim((SparkMax) io.leader()).setPosition(60.0 / 360 + 0.1);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double told = inputs.position;
    io.updateInputs(inputs);

    assertThat(Math.toDegrees(told)).isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(inputs.position))
        .as("then from its rotor")
        .isCloseTo(60, within(1e-3));
  }
}
