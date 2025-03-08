package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import frc.lib.Subsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.HumanInterface.StationData;

public class ClimberSubsystem extends Subsystem {
    // THESE ARE DUMMY VALUES!!!!! TODO: #8 Update algae pivot position values once determined.
    public static final Map<Subsystem.FieldPosition, Double> POSITIONS;
    static {
        POSITIONS = new EnumMap<>(Subsystem.FieldPosition.class);
        POSITIONS.put(Subsystem.FieldPosition.STARTING, 0.0);
        POSITIONS.put(Subsystem.FieldPosition.DRIVING, 0.0);
        POSITIONS.put(Subsystem.FieldPosition.CAGE, 0.0);
    }

    public ClimberSubsystem() {
        super(ClimberConstants.kClimberConfig, POSITIONS, StationData.getInstance().kClimberMech);

        setDefaultCommand(aim(FieldPosition.STARTING));
    }
}