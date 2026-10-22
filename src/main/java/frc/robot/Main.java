package frc.robot;

import org.wpilib.framework.RobotBase;

/** Where the program starts. */
public final class Main {
  public static void main(String... args) {
    // To run another step, change Step1OneMotor to its name, like Step2ReadASensor.
    RobotBase.startRobot(Step1OneMotor::new);
  }
}
