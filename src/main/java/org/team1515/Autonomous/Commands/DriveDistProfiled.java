package org.team1515.Autonomous.Commands;

import org.team1515.Autonomous.Drivetrain;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistProfiled extends CommandBase {
    private Drivetrain drivetrainSubsystem;
    private double direction;

    private double initalDist;
    private double distTraveled;
    private double maxSpeed;
    private ProfiledPIDController controller;

    private static double p = 5.5;
    private static double maxVel = 1.25;
    private static double maxAcc = 8;

    public DriveDistProfiled(Drivetrain drivetrainSubsystem, double targetDist, double direction) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.direction = direction;
        this.maxSpeed = 0.5 * SwerveConstants.Swerve.maxSpeed;

        controller = new ProfiledPIDController(p, 0, 0, new Constraints(maxVel, maxAcc));
        controller.setTolerance(0.04, 0.3);
        controller.setGoal(targetDist);

        SmartDashboard.putNumber("target dist", targetDist);
        addRequirements(drivetrainSubsystem);
    }
    public DriveDistProfiled(Drivetrain drivetrainSubsystem, double targetDist, double direction, double velocity, double acceleration) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.direction = direction;
        this.maxSpeed = 0.5 * SwerveConstants.Swerve.maxSpeed;

        controller = new ProfiledPIDController(p, 0, 0, new Constraints(velocity, acceleration));
        controller.setTolerance(0.04, 0.3);
        controller.setGoal(targetDist);

        SmartDashboard.putNumber("target dist", targetDist);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        this.initalDist = drivetrainSubsystem.getModulePositions()[0].distanceMeters;
        this.distTraveled = 0.0;
        controller.reset(distTraveled);
    }

    @Override
    public void execute() {
        distanceTraveled();
        double output = MathUtil.clamp(controller.calculate(distTraveled), -maxSpeed, maxSpeed);
        SmartDashboard.putNumber("output", output);
        SmartDashboard.putNumber("distance", distTraveled);
        drivetrainSubsystem.drive(new Translation2d(output * direction, 0.0), 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
    }

    /**
     * Gets the distance traveled by reading drive motor encoder
     */
    public void distanceTraveled() {
        distTraveled = Math.abs(drivetrainSubsystem.getModulePositions()[0].distanceMeters - initalDist);
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }
}