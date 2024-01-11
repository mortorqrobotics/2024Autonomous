package org.team1515.Autonomous.Commands;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.team1515.Autonomous.Drivetrain;
import org.team1515.Autonomous.RobotContainer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team1515.Autonomous.utils.Point;
import org.team1515.Autonomous.utils.bezierUtil;

import com.team364.swervelib.util.SwerveConstants;

import org.team1515.Autonomous.utils.Equation;

public class driveBezier extends CommandBase{
    
    private Drivetrain drivetrain;
    private ArrayList<Point> curve;
    private PIDController angleController;
    private double maxRotate;
    private ArrayList<Equation> derivativeEquation;
    private DoubleSupplier startAngle;
    private DoubleSupplier angle;
    private double t;
    private double ff = 0.0; // retune
    private double realTime;
    public driveBezier(Drivetrain drivetrain, ArrayList<Point> arr, DoubleSupplier theta, double t){
        this.drivetrain = drivetrain;
        this.curve = arr;
        this.angle = theta;
        this.t = t;

        this.maxRotate = 0.5 * SwerveConstants.Swerve.maxAngularVelocity;
        this.startAngle = () -> RobotContainer.gyro.getGyroscopeRotation().getRadians();
        angleController = new PIDController(2, 1, 0);
        // TODO retune PID
        angleController.setTolerance(Units.degreesToRadians(3));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        realTime = System.currentTimeMillis();
        derivativeEquation = bezierUtil.derivativeEquation(curve);
        
        addRequirements(drivetrain);
    }

    private double getAngle() {
        return startAngle.getAsDouble() + angle.getAsDouble();
    }

    @Override
    public void initialize() {
        angleController.setSetpoint(MathUtil.angleModulus(getAngle()));
        System.out.println("Start: " + MathUtil.angleModulus(getAngle()));
    }

    @Override
    public void execute(){

        double currentAngle = RobotContainer.gyro.getGyroscopeRotation().getRadians();
        double error = -MathUtil.angleModulus(currentAngle - angleController.getSetpoint());
        double rotation = (MathUtil.clamp(angleController.calculate(error + angleController.getSetpoint(), angleController.getSetpoint()) + (ff * Math.signum(-error)),
                -maxRotate, maxRotate));
        if (System.currentTimeMillis()-realTime < (t*1000)){
            double currentTime = (System.currentTimeMillis()-realTime)/(t*1000);
            double i = 0.0;
            double j = 0.0;
            for(Equation p : derivativeEquation){
                i+=p.applyX(currentTime);
                j+=p.applyY(currentTime); 
            }
            double mag = Math.sqrt(Math.pow(i,2)+Math.pow(j,2));
                
            drivetrain.drive(new Translation2d(i/mag,j/mag), rotation, true,true);
        }
        else{
            drivetrain.drive(new Translation2d(0,0), rotation, true,true);
        }

    }

    @Override
    public boolean isFinished(){
        return System.currentTimeMillis()-realTime >= (t*1000) && angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.drive(new Translation2d(0,0), 0, false, false);
    }

}
