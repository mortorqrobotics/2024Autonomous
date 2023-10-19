package org.team1515.Autonomous.Commands;

import org.team1515.Autonomous.Drivetrain;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveLine extends CommandBase {
    private final Drivetrain drivetrain;
    private double rot;
    private double i; //i vector component
    private double j; //j vector component
    private double ti; //initial time parameter
    private double tf; //final time parameter
    private double realTime; //actial system time
    private double maxSpeed;
    public driveLine(Drivetrain drivetrain, double rot, double i, double j, double ti, double tf) {
        this.drivetrain = drivetrain;
        this.rot = rot;
        this.i = i;
        this.j = j;
        this.ti = ti*1000;
        this.tf = tf*1000;
        realTime = System.currentTimeMillis();
        this.maxSpeed = (0.5/1.5) * SwerveConstants.Swerve.maxSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(new Translation2d(maxSpeed*i,maxSpeed*j),rot/(tf/1000-ti/1000),true,false);
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