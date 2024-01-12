package org.team1515.Autonomous.Commands;

import org.team1515.Autonomous.Drivetrain;
import org.team1515.Autonomous.utils.Point;

import com.team364.swervelib.util.SwerveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveSegment extends CommandBase {
    private final Drivetrain drivetrain;
    private double theta;
    private Point start;
    private Point end;
    private double t; //initial time parameter
    private double speed;
    private double startTime; //actial system time
    private double i;
    private double j;
    private Pose2d originalPose;
    double iError;
    double jError;
    
    public driveSegment(Drivetrain drivetrain, double theta, double speed, Point start, Point end, double t) {
        this.drivetrain = drivetrain;
        this.theta = theta;
        this.start = start;
        this.end = end;
        double dx = end.x-start.x;//change in x from start to end
        double dy = end.y-start.y;//change in y from start to end
        double mag = Math.sqrt(Math.pow(dx, 2)+Math.pow(dy, 2));//magnitude of the change vector
        this.i = dx/mag; //unit vector i component
        this.j = dy/mag; //unit vector j component
        this.t = t*1000;
        startTime = System.currentTimeMillis();
        this.speed = speed;
        addRequirements(drivetrain);
        this.iError = 0.0;
        this.jError = 0.0;


    }
    public driveSegment(Drivetrain drivetrain, double theta, double speed, Point start, Point end, double t, Pose2d pose) {
        this.drivetrain = drivetrain;
        this.theta = theta;
        this.start = start;
        this.end = end;
        double dx = end.x-start.x;//change in x from start to end
        double dy = end.y-start.y;//change in y from start to end
        double mag = Math.sqrt(Math.pow(dx, 2)+Math.pow(dy, 2));//magnitude of the change vector
        this.i = dx/mag; //unit vector i component
        this.j = dy/mag; //unit vector j component
        this.t = t*1000;
        startTime = System.currentTimeMillis();
        this.speed = speed;
        addRequirements(drivetrain);
        this.originalPose = pose;
        Pose2d currentPose = drivetrain.getOdometry();
        double originalX = originalPose.getX();
        double originalY = originalPose.getY();
        double projectedX = originalX+start.x;
        double projectedY = originalY+start.y;
        this.iError = projectedX-currentPose.getX();
        this.jError = projectedY-currentPose.getY();
        //adds vector to correct error (error/time driving)/speed
        this.i+=(iError/t)/speed;
        this.j+=(jError/t)/speed;
    }

    @Override
    public void initialize(){
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        drivetrain.drive(new Translation2d(speed*i,speed*j),theta/(t/1000),true,false);
        //System.out.println("i: " + i + " j: " + j + " speed: " + speed + " length: " + speed*t);
    }

    @Override
    public boolean isFinished() {
        //System.out.println("t: " + t);
        return System.currentTimeMillis()-startTime >= t;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.print("t: " + (System.currentTimeMillis()-startTime));
        System.out.println(" Speed: " + speed);
        //drivetrain.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
    }
}