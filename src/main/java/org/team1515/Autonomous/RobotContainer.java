// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1515.Autonomous;

import java.util.ArrayList;

import org.team1515.Autonomous.Commands.DefaultDriveCommand;
import org.team1515.Autonomous.Commands.driveBezier;
import org.team1515.Autonomous.utils.Gyroscope;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public static Gyroscope gyro;
  public static XboxController mainController;
  private Drivetrain drivetrain;

  public RobotContainer() {
    configureBindings();

    gyro = new Gyroscope();
    mainController = new XboxController(0);
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(drivetrain,
            () -> -modifyAxis(-mainController.getLeftY() * getRobotSpeed()),
            () -> -modifyAxis(-mainController.getLeftX() * getRobotSpeed()),
            () -> -modifyAxis(mainController.getRightX() * getRobotSpeed()),
            () -> Controls.DRIVE_ROBOT_ORIENTED.getAsBoolean()));

  }

  public static double getRobotSpeed() {
    return Controls.getLeftTrigger() ? 0.45 : 0.7;
    // return 0.7;
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.08);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

  public Command getAutonomousCommand() {

    ArrayList<Pair<Double,Double>> points = new ArrayList<Pair<Double,Double>>();
    points.add(new Pair(1,1));
    points.add(new Pair(-2,1));

    //return Commands.print("No autonomous command configured");
    return new driveBezier(drivetrain, points, 0, 3);
  }
}
