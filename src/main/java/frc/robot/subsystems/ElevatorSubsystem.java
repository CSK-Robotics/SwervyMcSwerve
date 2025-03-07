package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.HumanInterface.StationData;

public class ElevatorSubsystem extends Subsystem {
    // THESE ARE DUMMY VALUES!!!!! TODO: #11 Update elevator position values once
    // determined.
    public static final Map<FieldPosition, Double> POSITIONS;
    static {
        POSITIONS = new EnumMap<>(FieldPosition.class);
        POSITIONS.put(FieldPosition.STARTING, 0.0);
        POSITIONS.put(FieldPosition.DRIVING, 0.0);
        POSITIONS.put(FieldPosition.HPSTATION, 0.0);
        POSITIONS.put(FieldPosition.PROCESSOR, 0.0);
        POSITIONS.put(FieldPosition.L1, 0.0);
        POSITIONS.put(FieldPosition.L2, 0.0);
        POSITIONS.put(FieldPosition.L3, 0.0);
        POSITIONS.put(FieldPosition.L4, 0.0);
        POSITIONS.put(FieldPosition.NET, 0.0);
    }

    public ElevatorSubsystem() {
        super(ElevatorConstants.kElevatorConfig, POSITIONS, StationData.getInstance().kElevatorCarriage);

        setDefaultCommand(zero());
    }
}
