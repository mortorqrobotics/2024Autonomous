package org.team1515.Autonomous.Commands;

import java.util.ArrayList;

import org.team1515.Autonomous.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team1515.Autonomous.utils.Point;
import org.team1515.Autonomous.utils.bezierUtil;
import org.team1515.Autonomous.utils.CartesianPoint;
import org.team1515.Autonomous.utils.Equation;
import org.team1515.Autonomous.Commands.driveSegment;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class driveArcLength extends SequentialCommandGroup {
  /**
   * Creates a new driveArcLength.
   *

   */
  public driveArcLength(Drivetrain drivetrain, Point[] points, double t, double theta) {
    double length = bezierUtil.bezierLength(points);
    double segmentLength = length/points.length;
    double speed = length/t;
    double segmentT = segmentLength/speed;

    for(int i = 0; i<points.length-2;i++){
        addCommands(new driveSegment(drivetrain, theta, speed, points[i], points[i+1], segmentT));
    }
  }

}