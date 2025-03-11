package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.AngularSubsystem;
import frc.lib.subsystems.ISubsystem;
import frc.lib.subsystems.LinearSubsystem;
import frc.lib.subsystems.Subsystem;
import frc.lib.subsystems.Subsystem.FieldPosition;
import frc.lib.subsystems.Subsystem.FieldPositionValue;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.HumanInterface.StationData;

public class AlgaeSubsystem implements ISubsystem {
    public static final Map<FieldPosition, FieldPositionValue> ARM_POSITIONS;
    public static final Map<Subsystem.FieldPosition, FieldPositionValue> WHEEL_POSITIONS;
    static {
        ARM_POSITIONS = new EnumMap<>(FieldPosition.class);
        for (FieldPosition pos : FieldPosition.values()) {
            ARM_POSITIONS.put(pos, new FieldPositionValue());
        }

        WHEEL_POSITIONS = new EnumMap<>(Subsystem.FieldPosition.class);
        for (FieldPosition pos : FieldPosition.values()) {
            WHEEL_POSITIONS.put(pos, new FieldPositionValue());
        }
    }

    private final Subsystem m_arm, m_wheel;

    public AlgaeSubsystem() {
        m_arm = new AngularSubsystem(AlgaeConstants.kArmConfig, ARM_POSITIONS, StationData.getInstance().kAlgaeMech);
        m_wheel = new LinearSubsystem(AlgaeConstants.kWheelConfig, WHEEL_POSITIONS, null);

        // THESE ARE DUMMY VALUES!!!!! TODO: #8 Update algae pivot position values once
        // determined.
        ARM_POSITIONS.get(FieldPosition.STARTING).accept(null);
        ARM_POSITIONS.get(FieldPosition.DRIVING).accept(null);
        ARM_POSITIONS.get(FieldPosition.PROCESSOR).accept(null);
        ARM_POSITIONS.get(FieldPosition.L2).accept(null);
        ARM_POSITIONS.get(FieldPosition.L3).accept(null);
        ARM_POSITIONS.get(FieldPosition.NET).accept(null);

        WHEEL_POSITIONS.get(Subsystem.FieldPosition.STARTING).accept(null);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.DRIVING).accept(null);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.PROCESSOR).accept(null);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.L2).accept(null);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.L3).accept(null);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.NET).accept(null);
    }

    // Getters

    /**
     * Get the arm subsystem
     *
     * @return {@link frc.lib.subsystems.Subsystem}
     */
    public Subsystem getArm() {
        return m_arm;
    }

    /**
     * Get the wheel subsystem
     *
     * @return {@link frc.lib.subsystems.Subsystem}
     */
    public Subsystem getWheel() {
        return m_wheel;
    }

    // Commands

    /**
     * Set the goal of the arm
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command aim(FieldPosition goal) {
        return m_arm.aim(goal).alongWith(m_wheel.aim(goal));
    }

    /**
     * Release the with arm at set position
     *
     * @param goal Goal in meters
     * @param fire Fire event
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command score(FieldPosition goal, BooleanEvent fire) {
        return m_arm.score(goal, fire).alongWith(m_wheel.score(goal, fire));
    }

    /**
     * Intake from human player station
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command intake(FieldPosition position) {
        return m_arm.intake(position).alongWith(m_wheel.intake(position));
    }

    public Command zero() {
        return m_arm.zero().alongWith(m_wheel.zero());
    }

}