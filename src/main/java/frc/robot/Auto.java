package frc.robot;
import com.pathplanner.lib.config.RobotConfig;

import frc.robot.Constants.AutonomousPosition;
import frc.robot.Constants.Swerve;

public class Auto {
    SwerveModule[] swerveList;
    RobotConfig config;

    public Auto(SwerveModule[] swerveList) {
        this.swerveList = swerveList;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }      
    }

    public void drive(AutonomousPosition autonomousProfile) {
        // Execute pathplanner here to get into position:
        // Rotate until Limelight should be able to see the AprilTag
        // Detect limelight with targeting and move toward until area is certain size
    }

    private void keepTargetCentered() {

    }
}