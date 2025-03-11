package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import frc.lib.subsystems.AngularSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.HumanInterface.StationData;

public class ClimberSubsystem extends AngularSubsystem {
    public static final Map<FieldPosition, FieldPositionValue> POSITIONS;
    static {
        POSITIONS = new EnumMap<>(FieldPosition.class);
        for (FieldPosition pos : FieldPosition.values()) {
            POSITIONS.put(pos, new FieldPositionValue());
        }
    }

    public ClimberSubsystem() {
        super(ClimberConstants.kClimberConfig, POSITIONS, StationData.getInstance().kClimberMech);

        // THESE ARE DUMMY VALUES!!!!! TODO: #8 Update algae pivot position values once
        // determined.
        POSITIONS.get(FieldPosition.STARTING).accept(0.0);
        POSITIONS.get(FieldPosition.DRIVING).accept(0.0);
        POSITIONS.get(FieldPosition.CAGE).accept(135.0);

        setDefaultCommand(aim(FieldPosition.STARTING));
    }
}