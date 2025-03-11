package frc.robot.HumanInterface;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.subsystems.Subsystem.FieldPosition;
import frc.robot.Commands.Orchestrator;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CoralSubsystem;

public class StationData {
	public final Mechanism2d sideRobotView = new Mechanism2d(0.0, 0.0 /* canvas size */);
	public final MechanismRoot2d kElevatorBase, kClimberBase;
	public final MechanismLigament2d kElevatorTower, kElevatorCarriage, kCoralMech, kAlgaeMech, kClimberMech;

	private static StationData instance;

	public StationData(Orchestrator orchestrator) {
		// Supersystem
		kElevatorBase = sideRobotView.getRoot("Elevator Base", 0.0, 0.0); // Elevator position on chassis
		kElevatorTower = kElevatorBase.append(new MechanismLigament2d(
				"Elevator Tower", ElevatorConstants.kElevatorConfig.kBaseHeight, 90.0, 8.0,
				new Color8Bit(Color.kRed)));
		kElevatorCarriage = kElevatorTower.append(new MechanismLigament2d(
				"Elevator Carriage",
				ElevatorConstants.kElevatorConfig.kBaseHeight
						- ElevatorConstants.kElevatorConfig.kPositionOffset,
				-90,
				6,
				new Color8Bit(Color.kRed)));
		kCoralMech = kElevatorCarriage.append(
				new MechanismLigament2d(
						"Coral",
						CoralConstants.kAngularConfig.kRadius, // ArmConstants.kArmLength,
						CoralSubsystem.ARM_POSITIONS.get(FieldPosition.STARTING).m_value.get()
								.doubleValue(), // ArmConstants.kArmStartingAngle.in(Degrees),
						6, new Color8Bit(Color.kYellow)));
		kAlgaeMech = kElevatorCarriage.append(
				new MechanismLigament2d(
						"Algae",
						AlgaeConstants.kArmConfig.kRadius, // ArmConstants.kArmLength,
						AlgaeSubsystem.ARM_POSITIONS.get(FieldPosition.STARTING).m_value.get()
								.doubleValue(), // ArmConstants.kArmStartingAngle.in(Degrees),
						6, new Color8Bit(Color.kGreen)));

		// Climber
		kClimberBase = sideRobotView.getRoot("Climber Base", 0.0, 0.0); // Climber position on chassis
		kClimberMech = kClimberBase.append(
				new MechanismLigament2d(
						"Algae",
						ClimberConstants.kClimberConfig.kRadius, // ArmConstants.kArmLength,
						ClimberSubsystem.POSITIONS.get(FieldPosition.STARTING).m_value.get()
								.doubleValue(), // ArmConstants.kArmStartingAngle.in(Degrees),
						6, new Color8Bit(Color.kGreen)));
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
