// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1515.Autonomous;

import java.util.ArrayList;

import org.team1515.Autonomous.Commands.DefaultDriveCommand;
import org.team1515.Autonomous.Commands.driveBezier;
import org.team1515.Autonomous.utils.Gyroscope;
import org.team1515.Autonomous.utils.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public static Gyroscope gyro;
  public static XboxController mainController;
  public Drivetrain drivetrain;

  public RobotContainer() {
    gyro = new Gyroscope();
    mainController = new XboxController(0);
    drivetrain = new Drivetrain(new Pose2d());
    
    configureBindings();
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

    ArrayList<Point> points = new ArrayList<Point>();
    points.add(new Point(0.0,0.0));
    points.add(new Point(0,-0.5));
    points.add(new Point(-0.5,0));
    points.add(new Point(1.0,0));

    //return Commands.print("No autonomous command configured");
    return new driveBezier(drivetrain, points, 0, 3);
  }
}
