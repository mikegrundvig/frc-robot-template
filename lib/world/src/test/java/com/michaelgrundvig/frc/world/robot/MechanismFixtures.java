package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Shape;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * Synthetic mechanisms for the tests, one of each kind the audit's table lists, on the test robot's
 * body (a 28 x 24 in frame, 30 in tall, 60 kg). Numbers are chosen to check by hand.
 */
final class MechanismFixtures {
  private MechanismFixtures() {}

  static final Material ALUMINUM = Material.builder().name("aluminum").build();

  /** Turning about this, a joint whose zero points forward raises it as its position grows. */
  static final Vector3 PITCH_UP = new Vector3(0, -1, 0);

  static final Vector3 UP = Vector3.UP;

  /**
   * A link of a mass centered at a point in its joint's frame, as hard to turn as a box of a size
   * there, and that box its shape.
   */
  static Link link(String label, double kilograms, Translation3d cg, Translation3d size) {
    return Link.builder()
        .label(label)
        .mass(
            MassProperties.builder()
                .mass(Kilograms.of(kilograms))
                .cg(cg)
                .inertia(Inertia.box(Kilograms.of(kilograms), size))
                .build())
        .shapes(
            List.of(
                Shape.Box.builder()
                    .pose(new Pose3d(cg, Rotation3d.ZERO))
                    .size(size)
                    .material(ALUMINUM)
                    .build()))
        .build();
  }

  static Joint.Revolute revolute(
      String label,
      Part parent,
      Translation3d at,
      Vector3 axis,
      Optional<Range> range,
      double reference,
      Link link) {
    return Joint.Revolute.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .link(link)
        .range(range)
        .rest(reference)
        .build();
  }

  /**
   * A joint as a test's fixture has it, its motors taken off for the test to give it others: a
   * block says its motors once, and a joint re-wired keeps its motor.
   */
  static Joint.Revolute unmotored(Joint.Revolute joint) {
    return joint.driven(DrivenBuilder.builder(joint.driven()).motors(Optional.empty()).build());
  }

  static Joint.Prismatic prismatic(
      String label, Part parent, Translation3d at, Vector3 axis, Range range, Link link) {
    return Joint.Prismatic.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .link(link)
        .range(range)
        .rest(range.lower())
        .build();
  }

  /** The test robot with these mechanisms, and these cameras. */
  static RobotDefinition definition(
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras) {
    return RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
        .cameras(cameras)
        .joints(joints)
        .couplings(couplings)
        .drives(drives)
        .sensors(sensors)
        .build();
  }

  static RobotDefinition definition(
      List<Joint> joints, List<Coupling> couplings, List<Drive> drives, List<Sensor> sensors) {
    return definition(joints, couplings, drives, sensors, List.of());
  }

  static Gearing gearing(double ratio, double efficiency) {
    return Gearing.builder().ratio(ratio).efficiency(efficiency).build();
  }

  /** An encoder on one joint: a CANcoder's 4096 counts a turn if absolute, none given if not. */
  static Sensor.Encoder encoder(String label, Joint joint, double ratio, boolean absolute) {
    return encoder(label, joint, ratio, absolute, absolute ? Optional.of(4096) : Optional.empty());
  }

  static Sensor.Encoder encoder(
      String label, Joint joint, double ratio, boolean absolute, Optional<Integer> countsPerTurn) {
    return Sensor.Encoder.builder()
        .label(label)
        .terms(List.of(Transmission.Term.builder().joint(joint).ratio(ratio).build()))
        .absolute(absolute)
        .countsPerTurn(countsPerTurn)
        .build();
  }

  static Coupling.Linear linear(Joint leader, Joint follower, double ratio) {
    return Coupling.Linear.builder()
        .leader(leader)
        .follower(follower)
        .ratio(ratio)
        .offset(0)
        .build();
  }

  static Translation3d point(double x, double y, double z) {
    return new Translation3d(x, y, z);
  }

  static double meters(org.wpilib.units.measure.Distance distance) {
    return distance.in(Meters);
  }

  /** A joint's motors, through a gearing, as a drive of the robot's own. */
  static Drive motors(Joint.Revolute joint, Motor motor, int count, Gearing gearing) {
    return Drive.builder()
        .label(joint.label())
        .actuator(Actuator.Motors.builder().motor(motor).count(count).build())
        .transmission(Transmission.Geared.builder().joint(joint).gearing(gearing).build())
        .build();
  }

  /** A sliding joint's motors, through a gearing onto a drum, as a drive of the robot's own. */
  static Drive motors(
      Joint.Prismatic joint, Motor motor, int count, Gearing gearing, Distance pitchRadius) {
    return Drive.builder()
        .label(joint.label())
        .actuator(Actuator.Motors.builder().motor(motor).count(count).build())
        .transmission(
            Transmission.Drum.builder()
                .joint(joint)
                .gearing(gearing)
                .pitchRadius(pitchRadius)
                .build())
        .build();
  }

  /** What a robot's definition says it doesn't know: its notes, each its message. */
  static List<String> notes(RobotDefinition robot) {
    return robot.findings().stream()
        .filter(finding -> finding.level() == Finding.Level.NOTE)
        .map(Finding::message)
        .toList();
  }

  /** What a robot's definition doubts, each its message. */
  static List<String> doubts(RobotDefinition robot) {
    return robot.findings().stream()
        .filter(finding -> finding.level() == Finding.Level.DOUBT)
        .map(Finding::message)
        .toList();
  }

  /** The scope of each note, in order. */
  static List<Finding.Scope> noteScopes(RobotDefinition robot) {
    return robot.findings().stream()
        .filter(finding -> finding.level() == Finding.Level.NOTE)
        .map(Finding::scope)
        .toList();
  }

  /**
   * A control for a joint held at positions with no absolute encoder: stowed by hand at power on,
   * its feedback {@code kP}.
   */
  static MechanismControl stowed(double kP) {
    return MechanismControl.position(kP).withHoming(MechanismControl.Homing.assumeAtRest());
  }
}
