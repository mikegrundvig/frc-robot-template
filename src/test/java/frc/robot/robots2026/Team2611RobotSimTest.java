package frc.robot.robots2026;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.Joint;
import frc.robot.RobotSim;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.JointSimulation;
import frc.robot.subsystems.mechanisms.JointMechanism;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.simulation.GamepadSim;

/**
 * 2611's 2026 robot end to end in the template's simulator: their definition, their behavior
 * ({@link Team2611Actions}) bound to two gamepads, run by the robot program's own loop, the drive
 * and localization included. The behavior design's checkpoint C (docs/design/robot-behavior.md).
 * Checked against the simulator's truth throughout: the intake's roller is never driven up to speed
 * while the pivot is short of "run". A test fixture, not 2026 play.
 */
class Team2611RobotSimTest {
  /** The blue hub's center, as their code has it (Constants.java:178-182: 182.11 in, 158.84 in). */
  static final Translation2d HUB = new Translation2d(Inches.of(182.11), Inches.of(158.84));

  /**
   * Their paths' times, standing in for a path follower (the path tool is the team's open
   * decision): each path's length between its points, at its own speed and acceleration limits, as
   * if straight (deploy/pathplanner/paths: 5.00 m, 5.56 m, 11.89 m at 3, 3 and 2.2 m/s, 5 m/s²).
   */
  static final Map<String, Double> PATH_SECONDS =
      Map.of("EXP Left 1", 2.27, "EXP Left 2", 2.45, "EXP Left 3", 5.84);

  @RegisterExtension final RobotSim sim = new RobotSim();

  private GamepadSim operator;
  private Team2611Actions actions;
  private JointMechanism pivot;
  private JointMechanism roller;
  private JointMechanism fullSend;
  private JointSimulation truth;

  /** The fastest the roller was turning last loop, and whether it ever sped up short of "run". */
  private double rollerWas;

  private boolean ruleBroken;

  @BeforeEach
  void build() {
    sim.build(Team2611.CONFIG, SensorErrors.NONE);
    operator = new GamepadSim(1);
    actions = new Team2611Actions(sim.joints(), sim.scheduler(), this::aimError);
    actions.bind(new CommandGamepad(sim.scheduler(), 0), new CommandGamepad(sim.scheduler(), 1));
    pivot = sim.joints().of(Team2611.PIVOT);
    roller = sim.joints().of(Team2611.INTAKE);
    fullSend = sim.joints().of(Team2611.FULL_SEND);
    truth = Objects.requireNonNull(sim.world().joints());
  }

  /**
   * How far off aim the robot is, in degrees, from where it thinks it is: their shooter faces the
   * robot's back (inferred: their paths end facing away from the hub to shoot, EXP Left 2 at 135°).
   */
  private double aimError() {
    Pose2d pose = sim.estimate().pose();
    Translation2d toHub = HUB.minus(pose.getTranslation());
    Rotation2d bearing = new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()));
    Rotation2d shooting = pose.getRotation().plus(new Rotation2d(Math.PI));
    return shooting.minus(bearing).getDegrees();
  }

  /** Robot loops for a time, checking the interlock against the simulator's truth each loop. */
  private void run(double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      sim.loop();
      double speed = Math.abs(truth.velocity(Team2611.INTAKE));
      if (Math.toDegrees(truth.position(Team2611.PIVOT)) < 110 && speed > rollerWas + 0.5) {
        ruleBroken = true;
      }
      rollerWas = speed;
    }
  }

  private void press(Runnable change) {
    change.run();
    operator.notifyNewData();
    sim.driver().notifyNewData();
  }

  /** How fast a joint really turns, in rpm. */
  private double rpm(Joint joint) {
    return RPM.convertFrom(truth.velocity(joint), RadiansPerSecond);
  }

  @Test
  void theOperatorsIntakeDeploysThePivotAndTheRollerWaitsForIt() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setRightTriggerAxis(1));
    run(0.1);
    String held = roller.heldBy();
    run(1.4);
    double intaking = rpm(Team2611.INTAKE);
    press(() -> operator.setRightTriggerAxis(0));
    run(1);

    assertThat(held).isEqualTo("intake runs only when pivot past run");
    assertThat(intaking).as("near 2000 rpm, its free speed").isGreaterThan(1900);
    assertThat(Math.abs(rpm(Team2611.INTAKE))).as("let go: stopped").isLessThan(10);
    assertThat(Math.toDegrees(truth.position(Team2611.PIVOT)))
        .as("and held out")
        .isCloseTo(130, within(2.0));
    assertThat(ruleBroken).isFalse();
  }

  /** Their outtake button ignored the guard their auto had; the interlock holds it too. */
  @Test
  void outtakingWithThePivotStowedIsHeld() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setLeftTriggerAxis(1));
    run(1);

    assertThat(roller.heldBy()).isEqualTo("intake runs only when pivot past run");
    assertThat(Math.abs(rpm(Team2611.INTAKE))).isLessThan(10);
    assertThat(ruleBroken).isFalse();
  }

  /** Out on one button and back in on another: the retract they never had. */
  @Test
  void theOperatorDeploysAndRetractsThePivot() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setFaceDownButton(true));
    run(0.1);
    press(() -> operator.setFaceDownButton(false));
    run(1.5);
    double out = Math.toDegrees(truth.position(Team2611.PIVOT));
    press(() -> operator.setFaceRightButton(true));
    run(0.1);
    press(() -> operator.setFaceRightButton(false));
    run(1.5);

    assertThat(out).isCloseTo(130, within(2.0));
    assertThat(Math.toDegrees(truth.position(Team2611.PIVOT))).isCloseTo(1.5, within(2.0));
    assertThat(pivot.lastOutcome()).startsWith("pivot → in: ARRIVED in ");
  }

  /**
   * Holding shoot, facing the wrong way: the shooter spins up and nothing feeds, blocked by aim.
   * The driver turns the robot until the shooter faces the hub, and it feeds; turned away, it
   * pauses, saying why; the right trigger feeds anyway, the manual override.
   */
  @Test
  void shootingFeedsOnlyWhenAimedAndSaysWhyNot() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> sim.driver().setLeftBumperButton(true));
    run(1);
    double unaimed = rpm(Team2611.FULL_SEND);
    var blocking = actions.readyToShoot().blocking();
    double shooter = rpm(Team2611.SHOOTER);
    press(() -> sim.driver().setRightX(0.5));
    for (int i = 0; i < RobotLoop.loops(4) && Math.abs(aimError()) > 1.5; i++) {
      run(0.02);
    }
    press(() -> sim.driver().setRightX(0));
    run(1);
    double aimed = rpm(Team2611.FULL_SEND);
    press(() -> sim.driver().setRightX(0.5));
    run(0.4);
    press(() -> sim.driver().setRightX(0));
    run(0.5);
    double turnedAway = rpm(Team2611.FULL_SEND);
    String why = fullSend.lastOutcome();
    press(() -> sim.driver().setRightTriggerAxis(1));
    run(1);
    double overridden = rpm(Team2611.FULL_SEND);

    assertThat(shooter).isCloseTo(Team2611Actions.SHOT_RPM, within(0.03 * 3000));
    assertThat(unaimed).isCloseTo(0, within(5.0));
    assertThat(blocking).containsExactly("aimed");
    assertThat(aimed).isCloseTo(1000, within(30.0));
    assertThat(turnedAway).isCloseTo(0, within(5.0));
    assertThat(why)
        .startsWith("at feeding: CANCELED after ")
        .endsWith(": ready to shoot is false: aimed");
    assertThat(overridden).as("the override feeds unaimed").isCloseTo(1000, within(30.0));
  }

  /**
   * Left Double in autonomous: the pivot out, the intake along the first two paths, a three-second
   * shot, the intake along the third, and shooting until auto ends, with paths standing in as time
   * on a still drive. The stand-in follower leaves the robot aimed, as their paths end facing the
   * hub's way. Every step ends as it should, and the interlock holds throughout.
   */
  @Test
  void leftDoubleRunsItsStepsInsideAuto() {
    Team2611Actions auto = new Team2611Actions(sim.joints(), sim.scheduler(), () -> 0);
    Function<String, Command> path =
        name ->
            sim.drive()
                .run(
                    co -> {
                      for (int i = 0;
                          i < RobotLoop.loops(Objects.requireNonNull(PATH_SECONDS.get(name)));
                          i++) {
                        sim.drive().runVelocity(new ChassisVelocities(0, 0, 0));
                        co.yield();
                      }
                    })
                .named(name + " (stand-in)");
    Command leftDouble = auto.leftDouble(path);
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    run(0.02);
    sim.scheduler().schedule(leftDouble);

    run(1.0); // the pivot out: 0.6 s or so
    String deployed = pivot.lastOutcome();
    run(2.0); // along the first path, the intake running
    double alongFirst = rpm(Team2611.INTAKE);
    run(4.0); // t = 7 s: the shot, after both paths (4.7 s of them, and the pivot's)
    double shooting = rpm(Team2611.FULL_SEND);
    run(4.0); // t = 11 s: along the third path, the shot done
    double alongThird = rpm(Team2611.INTAKE);
    double betweenShots = rpm(Team2611.FULL_SEND);
    run(6.0); // t = 17 s: the last shot, until auto ends
    double lastShot = rpm(Team2611.FULL_SEND);

    assertThat(deployed).matches("pivot → out: ARRIVED in 0\\.\\d\\d s");
    assertThat(alongFirst).isGreaterThan(1900);
    assertThat(shooting).isCloseTo(1000, within(30.0));
    assertThat(alongThird).isGreaterThan(1900);
    assertThat(betweenShots).isCloseTo(0, within(5.0));
    assertThat(lastShot).isCloseTo(1000, within(30.0));
    assertThat(sim.scheduler().isRunning(leftDouble)).as("shooting until auto ends").isTrue();
    assertThat(ruleBroken).isFalse();
  }
}
