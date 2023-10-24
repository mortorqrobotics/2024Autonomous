package org.team1515.Autonomous.Commands;

import java.util.ArrayList;
import java.util.function.DoubleFunction;

import org.team1515.Autonomous.Drivetrain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1515.Autonomous.utils.bezierUtil;

public class driveBezier extends CommandBase{
    
    private Drivetrain drivetrain;
    private ArrayList<Pair<Double, Double>> curve;
    private ArrayList<Pair<Double, Double>> derivativeCurve;
    private ArrayList<Pair<DoubleFunction<Double>, DoubleFunction<Double>>> derivativeEquation;
    private double theta;
    private double t;
    private double realTime;
    public driveBezier(Drivetrain drivetrain, ArrayList<Pair<Double, Double>> arr,double theta, double t){
        this.drivetrain = drivetrain;
        this.curve = arr;
        this.theta = theta;
        this.t = t*1000.0;
        realTime = System.currentTimeMillis();
        addRequirements(drivetrain);
        derivativeCurve = bezierUtil.derivativeBezier(curve);
        derivativeEquation = bezierUtil.bezierEquation(derivativeCurve);
    }

    @Override
    public void execute(){
        double currentTime = (System.currentTimeMillis()-realTime)/t;
        double i = 0.0;
        double j = 0.0;
        for(Pair<DoubleFunction<Double>, DoubleFunction<Double>>p : derivativeEquation){
            i+=p.getFirst().apply(currentTime);
            j+=p.getSecond().apply(currentTime);
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
