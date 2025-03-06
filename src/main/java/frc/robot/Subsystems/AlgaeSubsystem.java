package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends EndEffectorSubsystem {
    private static final Map<FieldPosition, Double> POSITIONS;
    static {
        POSITIONS = new EnumMap<>(FieldPosition.class);
        POSITIONS.put(FieldPosition.STARTING, 0.0);
        POSITIONS.put(FieldPosition.DRIVING, 0.0);
        POSITIONS.put(FieldPosition.PROCESSOR, 0.0);
        POSITIONS.put(FieldPosition.L2, 0.0);
        POSITIONS.put(FieldPosition.L3, 0.0);
        POSITIONS.put(FieldPosition.NET, 0.0);
    }

    public AlgaeSubsystem() {
        super(AlgaeConstants.instance, POSITIONS);
    }
}