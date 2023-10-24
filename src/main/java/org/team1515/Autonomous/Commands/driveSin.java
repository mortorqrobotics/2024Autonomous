package org.team1515.Autonomous.Commands;

import org.team1515.Autonomous.Drivetrain;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveSin extends CommandBase {
    private final Drivetrain drivetrain;
    private double ti; //initial time parameter
    private double tf; //final time parameter
    private double theta;
    private double realTime; //actial system time
    private double i; //i vector component of r'
    private double j; //i vector component of r'
    private double maxSpeed = (0.5/1.5) * SwerveConstants.Swerve.maxSpeed;
    public driveSin(Drivetrain drivetrain, double ti, double tf, double theta) {
        this.drivetrain = drivetrain;
        this.ti = ti*1000;
        this.tf = tf*1000;
        this.theta = theta;
        realTime = System.currentTimeMillis();
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        i = ti/1000+(System.currentTimeMillis()-realTime);
        j = Math.sin(ti/1000+(System.currentTimeMillis()-realTime));
        drivetrain.drive(new Translation2d(maxSpeed*i,maxSpeed*j), theta/(tf/1000-ti/1000), true,true);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis()-realTime >= tf-ti;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
    }
}