package org.team1515.Autonomous.Commands;

import java.util.ArrayList;

import org.team1515.Autonomous.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team1515.Autonomous.utils.Point;
import org.team1515.Autonomous.utils.bezierUtil;
import org.team1515.Autonomous.utils.Equation;

public class driveBezier extends CommandBase{
    
    private Drivetrain drivetrain;
    private ArrayList<Point> curve;
    private ArrayList<Point> derivativeCurve;
    private ArrayList<Equation> derivativeEquation;
    private double theta;
    private double t;
    private double realTime;
    public driveBezier(Drivetrain drivetrain, ArrayList<Point> arr,double theta, double t){
        this.drivetrain = drivetrain;
        this.curve = arr;
        this.theta = theta;
        this.t = t*1000.0;
        realTime = System.currentTimeMillis();
        derivativeCurve = bezierUtil.derivativeBezier(curve);
        derivativeEquation = bezierUtil.bezierEquation(derivativeCurve);
        
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        double currentTime = (System.currentTimeMillis()-realTime)/t;
        double i = 0.0;
        double j = 0.0;
        for(Equation p : derivativeEquation){
            i+=p.applyX(currentTime);
            j+=p.applyY(currentTime);
        }
        drivetrain.drive(new Translation2d(i,j), theta/(t/1000), true,true);

    }

    @Override
    public boolean isFinished(){
        return System.currentTimeMillis()-realTime >= t;
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.drive(new Translation2d(0,0), 0, false, false);
    }

}
