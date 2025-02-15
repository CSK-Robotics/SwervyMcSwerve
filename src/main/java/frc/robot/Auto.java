package frc.robot;
import com.pathplanner.lib.config.RobotConfig;
import frc.robot.Constants.AutonomousData;
import frc.robot.Constants.Swerve;

public class Auto {
    SwerveModule[] swerveList;
    RobotConfig config;

    public Auto(SwerveModule[] swerveList) {
        this.swerveList = swerveList;
    }

    public void drive(AutonomousData autonomousProfile) {
        LimelightHelpers.setPipelineIndex("", autonomousProfile.pipelineID);
        // Execute pathplanner here to get into position:
        // Rotate until Limelight should be able to see the AprilTag:
        var currentRotation = 0;
        while (!LimelightHelpers.getTV("")) { // getTV tells us if we even have a target
            // Since we don't have a target, we need to rotate until we do:
            
            currentRotation += 1;
        }
        // Detect limelight with targeting and move toward until area is certain size:
    }

    private double calculateTargetError() {
        
        return 0.0;
    }
}