package org.team1515.Autonomous.Commands;

import org.team1515.Autonomous.RobotContainer;
import org.team1515.Autonomous.Drivetrain;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RotateToAngle extends CommandBase {
    private Drivetrain drivetrainSubsystem;
    // l
    private PIDController angleController;
    private double maxRotate;

    private double ff = 0.0; // retune

    /**
     * Align robot with the target using the limelight
     */
    public RotateToAngle(Drivetrain drivetrainSubsystem, double angle) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.maxRotate = 0.5 * SwerveConstants.Swerve.maxAngularVelocity;

        angleController = new PIDController(2, 0, 0);
        // TODO retune PID
        angleController.setTolerance(Units.degreesToRadians(3.5));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setSetpoint(angle);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double error = MathUtil.angleModulus(RobotContainer.gyro.getGyroscopeRotation().getRadians())
                - drivetrainSubsystem.getRealZero().getRadians();
        double rotation = (MathUtil.clamp(angleController.calculate(error, 0.0) + (ff * Math.signum(-error)),
                -maxRotate, maxRotate));
        drivetrainSubsystem.drive(new Translation2d(0.0, 0.0), rotation, true, true);
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
}