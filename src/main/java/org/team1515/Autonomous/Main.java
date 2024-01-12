// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1515.Autonomous;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.ArrayList;

import org.team1515.Autonomous.utils.Point;
import org.team1515.Autonomous.utils.Equation;
import org.team1515.Autonomous.utils.bezierUtil;
class Main{
// public final class Main {
//   private Main() {}

  public static void main(String[] args) {
    RobotBase.startRobot(Robot::new);
  }
}
