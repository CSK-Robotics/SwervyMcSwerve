package frc.robot.Subsystems;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.HumanInterface.StationData;
import frc.robot.Subsystems.Subsystem.FieldPosition;

public class AlgaeSubsystem implements ISubsystem{
    // THESE ARE DUMMY VALUES!!!!! TODO: #8 Update algae pivot position values once determined.
    public static final Map<Subsystem.FieldPosition, Double> ARM_POSITIONS;
    public static final Map<Subsystem.FieldPosition, Double> WHEEL_POSITIONS;
    static {
        ARM_POSITIONS = new EnumMap<>(Subsystem.FieldPosition.class);
        ARM_POSITIONS.put(Subsystem.FieldPosition.STARTING, 0.0);
        ARM_POSITIONS.put(Subsystem.FieldPosition.DRIVING, 0.0);
        ARM_POSITIONS.put(Subsystem.FieldPosition.PROCESSOR, 0.0);
        ARM_POSITIONS.put(Subsystem.FieldPosition.L2, 0.0);
        ARM_POSITIONS.put(Subsystem.FieldPosition.L3, 0.0);
        ARM_POSITIONS.put(Subsystem.FieldPosition.NET, 0.0);

        WHEEL_POSITIONS = new EnumMap<>(Subsystem.FieldPosition.class);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.STARTING, 0.0);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.DRIVING, 0.0);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.PROCESSOR, 0.5);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.L2, -0.5);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.L3, -0.5);
        WHEEL_POSITIONS.put(Subsystem.FieldPosition.NET, 0.8);
    }

    private final Subsystem m_arm, m_wheel;
    public AlgaeSubsystem() {
        m_arm = new Subsystem(AlgaeConstants.kArmConfig, ARM_POSITIONS, StationData.getInstance().kAlgaeMech);
        m_wheel = new Subsystem(AlgaeConstants.kWheelConfig, WHEEL_POSITIONS, null);
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