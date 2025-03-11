package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import frc.lib.subsystems.LinearSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.HumanInterface.StationData;

public class ElevatorSubsystem extends LinearSubsystem {
    public static final Map<FieldPosition, FieldPositionValue> POSITIONS;
    static {
        POSITIONS = new EnumMap<>(FieldPosition.class);
        for (FieldPosition pos : FieldPosition.values()) {
            POSITIONS.put(pos, new FieldPositionValue());
        }
    }

    public ElevatorSubsystem() {
        super(ElevatorConstants.kElevatorConfig, POSITIONS, StationData.getInstance().kElevatorCarriage);

        // THESE ARE DUMMY VALUES!!!!! TODO: #11 Update elevator position values once
        // determined.
        POSITIONS.get(FieldPosition.STARTING).accept(0.0);
        POSITIONS.get(FieldPosition.DRIVING).accept(0.0);
        POSITIONS.get(FieldPosition.HPSTATION).accept(0.0);
        POSITIONS.get(FieldPosition.PROCESSOR).accept(0.0);
        POSITIONS.get(FieldPosition.L1).accept(6.0);
        POSITIONS.get(FieldPosition.L2).accept(12.0);
        POSITIONS.get(FieldPosition.L3).accept(18.0);
        POSITIONS.get(FieldPosition.L4).accept(24.0);
        POSITIONS.get(FieldPosition.NET).accept(26.0);

        setDefaultCommand(zero());
    }
}
