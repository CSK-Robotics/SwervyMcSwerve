package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.HumanInterface.StationData;
import frc.robot.Subsystems.Subsystem.FieldPosition;


public class CoralSubsystem implements ISubsystem {
    // THESE ARE DUMMY VALUES!!!!! TODO: #10 Update coral arm position values once determined.
    public static final Map<FieldPosition, Double> ARM_POSITIONS;
    public static final Map<Subsystem.FieldPosition, Double> WHEEL_POSITIONS;
    static {
        ARM_POSITIONS = new EnumMap<>(FieldPosition.class);
        ARM_POSITIONS.put(FieldPosition.STARTING, 0.0);
        ARM_POSITIONS.put(FieldPosition.DRIVING, 0.0);
        ARM_POSITIONS.put(FieldPosition.HPSTATION, 0.0);
        ARM_POSITIONS.put(FieldPosition.L1, 0.0);
        ARM_POSITIONS.put(FieldPosition.L2, 0.0);
        ARM_POSITIONS.put(FieldPosition.L3, 0.0);
        ARM_POSITIONS.put(FieldPosition.L4, 0.0);

        WHEEL_POSITIONS = new EnumMap<>(Subsystem.FieldPosition.class);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.STARTING, 0.0);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.DRIVING, 0.0);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.PROCESSOR, 0.5);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.L2, -0.5);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.L3, -0.5);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.NET, 0.8);
    }

    private final Subsystem m_arm, m_wheel;
    public CoralSubsystem() {
        m_arm = new Subsystem(CoralConstants.kArmConfig, ARM_POSITIONS, StationData.getInstance().kCoralMech);
        m_wheel = new Subsystem(CoralConstants.kWheelConfig, WHEEL_POSITIONS, null);
    }

    // Getters

    /**
     * Get the arm subsystem
     *
     * @return {@link frc.robot.Subsystems.Subsystem}
     */
    public Subsystem getArm() {
        return m_arm;
    }

    /**
     * Get the wheel subsystem
     *
     * @return {@link frc.robot.Subsystems.Subsystem}
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