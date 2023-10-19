package org.team1515.Autonomous;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controls {
    public static final Trigger DRIVE_ROBOT_ORIENTED = new Trigger(RobotContainer.mainController::getLeftBumper);
    public static final Trigger RESET_GYRO = new Trigger(RobotContainer.mainController::getBackButton);
    public static boolean getRightTrigger() {
        return RobotContainer.mainController.getRightTriggerAxis() >= 0.250;
    }

    public static boolean getLeftTrigger() {
        return RobotContainer.mainController.getLeftTriggerAxis() >= 0.250;
    }


}
