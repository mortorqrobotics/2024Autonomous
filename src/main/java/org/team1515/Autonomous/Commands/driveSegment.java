package org.team1515.Autonomous.Commands;

import org.team1515.Autonomous.Drivetrain;
import org.team1515.Autonomous.utils.CartesianPoint;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveSegment extends CommandBase {
    private final Drivetrain drivetrain;
    private double theta;
    private CartesianPoint start;
    private CartesianPoint end;
    private double t; //initial time parameter
    private double speed;
    private double startTime; //actial system time
    private double i;
    private double j;
    
    public driveSegment(Drivetrain drivetrain, double theta, double speed, CartesianPoint start, CartesianPoint end, double t) {
        this.drivetrain = drivetrain;
        this.theta = theta;
        this.start = start;
        this.end = end;
        double dx = end.getX()-start.getX();//change in x from start to end
        double dy = end.getY()-start.getY();//change in y from start to end
        double mag = Math.sqrt(Math.pow(dx, 2)+Math.pow(dy, 2));//magnitude of the change vector
        this.i = dx/mag; //unit vector i component
        this.j = dy/mag; //unit vector j component
        this.t = t*1000;
        startTime = System.currentTimeMillis();
        this.speed = speed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(new Translation2d(speed*i,speed*j),theta/(t/1000),true,false);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis()-startTime >= t;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
    }
}