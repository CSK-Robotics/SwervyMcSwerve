package frc.robot.HumanInterface;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Commands.Orchestrator;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.Subsystem.FieldPosition;

public class StationData {
    public final Mechanism2d sideRobotView = new Mechanism2d(0.0, 0.0 /* canvas size */);
    public final MechanismRoot2d kChassis;
    public final MechanismLigament2d kElevatorBase, kElevatorCarriage, kCoralMech, kAlgaeMech;

    private static StationData instance;

    public StationData(Orchestrator orchestrator) {
        kChassis = sideRobotView.getRoot("Chassis", 0.0, 0.0); // Elevator position on chassis
        kElevatorBase = kChassis.append(new MechanismLigament2d(
                "Elevator Intermediate Stage", ElevatorConstants.kElevatorConfig.kBaseHeight, 90.0, 8.0,
                new Color8Bit(Color.kRed)));
        kElevatorCarriage = kElevatorBase.append(new MechanismLigament2d(
                "Elevator",
                ElevatorConstants.kElevatorConfig.kBaseHeight - ElevatorConstants.kElevatorConfig.kCarriageGroundOffset, 
                -90,
                6,
                new Color8Bit(Color.kRed)));
        kCoralMech = kElevatorCarriage.append(
                new MechanismLigament2d(
                        "Coral",
                        CoralConstants.kArmConfig.kArmLength, // ArmConstants.kArmLength,
                        CoralSubsystem.ARM_POSITIONS.get(FieldPosition.STARTING), // ArmConstants.kArmStartingAngle.in(Degrees),
                        6,
                        new Color8Bit(Color.kYellow)));
        kAlgaeMech = kElevatorCarriage.append(
                new MechanismLigament2d(
                        "Algae",
                        AlgaeConstants.kArmConfig.kArmLength, // ArmConstants.kArmLength,
                        AlgaeSubsystem.ARM_POSITIONS.get(FieldPosition.STARTING), // ArmConstants.kArmStartingAngle.in(Degrees),
                        6,
                        new Color8Bit(Color.kGreen)));
    }

    public static final StationData getInstance(Orchestrator orchestrator) {
        if (instance == null) {
            instance = new StationData(orchestrator);
        }
        return instance;
    }

    public static final StationData getInstance() {
        return instance;
    }
}
