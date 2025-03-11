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
import frc.robot.Constants.CoralConstants;
import frc.robot.HumanInterface.StationData;

public class CoralSubsystem implements ISubsystem {
    // THESE ARE DUMMY VALUES!!!!! TODO: #10 Update coral arm position values once
    // determined.
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

    public CoralSubsystem() {
        ARM_POSITIONS.get(FieldPosition.STARTING).accept(0.0);
        ARM_POSITIONS.get(FieldPosition.DRIVING).accept(0.0);
        ARM_POSITIONS.get(FieldPosition.HPSTATION).accept(0.0);
        ARM_POSITIONS.get(FieldPosition.L1).accept(0.0);
        ARM_POSITIONS.get(FieldPosition.L2).accept(0.0);
        ARM_POSITIONS.get(FieldPosition.L3).accept(0.0);
        ARM_POSITIONS.get(FieldPosition.L4).accept(0.0);

        WHEEL_POSITIONS.get(Subsystem.FieldPosition.STARTING).accept(0.0);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.DRIVING).accept(0.0);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.PROCESSOR).accept(0.5);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.L2).accept(-0.5);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.L3).accept(-0.5);
        WHEEL_POSITIONS.get(Subsystem.FieldPosition.NET).accept(0.8);

        m_arm = new AngularSubsystem(CoralConstants.kAngularConfig, ARM_POSITIONS, StationData.getInstance().kCoralMech);
        m_wheel = new LinearSubsystem(CoralConstants.kWheelConfig, WHEEL_POSITIONS, null);
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
        return m_wheel.score(goal, fire).deadlineFor(m_arm.score(goal, fire)).andThen(zero());
    }

    /**
     * Intake from human player station
     *
     * @param goal Goal in meters
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command intake(FieldPosition position) {
        return m_wheel.intake(position).deadlineFor(m_arm.intake(position)).andThen(aim(FieldPosition.DRIVING));
    }

    /**
     * Zero the subsystem
     *
     * @return {@link edu.wpi.first.wpilibj2.command.Command}
     */
    public Command zero() {
        return m_arm.zero().alongWith(m_wheel.zero());
    }
}