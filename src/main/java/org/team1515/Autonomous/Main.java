// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1515.Autonomous;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.ArrayList;

import org.team1515.Autonomous.utils.Point;
import org.team1515.Autonomous.utils.Equation;
import org.team1515.Autonomous.utils.bezierUtil;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    // RobotBase.startRobot(Robot::new);
    ArrayList<Point> points = new ArrayList<Point>();
    points.add(new Point(0.0,0.0));
    points.add(new Point(0.5,1));
    points.add(new Point(1.0, 0.0));
    points.add(new Point(1.5,-1));
    points.add(new Point(2.0,0.0));
    
  //   ArrayList<Equation> eq = bezierUtil.bezierEquation(points);
  //   for(double num = 0.0; num<1.0; num+=0.1){
  //     double i = 0.0;
  //     double j = 0.0;
  //     for(Equation p : eq){
  //       i+=p.applyX(num);
  //       j+=p.applyY(num); 
  //     }
  //     System.out.println("t:"+Double.toString(num)+" x:"+Double.toString(i)+" y:"+Double.toString(j));
  // }
}
}
