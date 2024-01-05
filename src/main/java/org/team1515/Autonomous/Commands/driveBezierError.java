package org.team1515.Autonomous.Commands;

import java.util.ArrayList;

import org.team1515.Autonomous.Drivetrain;
import org.team1515.Autonomous.utils.Equation;
import org.team1515.Autonomous.utils.Point;
import org.team1515.Autonomous.utils.bezierUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class driveBezierError extends CommandBase{
    
    private Drivetrain drivetrain;
    private ArrayList<Point> curve;
    private ArrayList<Equation> derivativeEquation;
    private ArrayList<Equation> bezierEquation;
    private double theta;
    private double t;
    private double startTime;
    private Pose2d startPose;
    public driveBezierError(Drivetrain drivetrain, ArrayList<Point> arr,double theta, double t){
        this.drivetrain = drivetrain;
        this.curve = arr;
        this.theta = theta;
        this.t = t;
        this.startPose=drivetrain.getOdometry();
        startTime = System.currentTimeMillis();
        derivativeEquation = bezierUtil.derivativeEquation(curve);
        bezierEquation = bezierUtil.bezierEquation(curve);
        
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        double currentTime = (System.currentTimeMillis()-startTime)/(t*1000);
        double deri = 0.0;
        double derj = 0.0;
        for(Equation p : derivativeEquation){
            deri+=p.applyX(currentTime);
            derj+=p.applyY(currentTime); 
        }
        double posi = 0.0;
        double posj = 0.0;
        for(Equation p : bezierEquation){
            posi+=p.applyX(currentTime);
            posj+=p.applyY(currentTime); 
        }
        Pose2d curPose = drivetrain.getOdometry();
        double xError= (startPose.getX()+posi)-curPose.getX();
        double yError= (startPose.getY()+posj)-curPose.getY();
        double mag = Math.sqrt(Math.pow(deri,2)+Math.pow(derj,2));
        double i=(deri/mag)+xError;
        double j=(derj/mag)+yError;
        drivetrain.drive(new Translation2d(i,j), theta/(t), true,true);


    }

    @Override
    public boolean isFinished(){
        return System.currentTimeMillis()-startTime >= (t*1000);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.drive(new Translation2d(0,0), 0, false, false);
    }

}
