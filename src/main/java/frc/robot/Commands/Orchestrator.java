package frc.robot.Commands;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.Subsystem.FieldPosition;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

public class Orchestrator {
    public enum ReefPosition {
        L1,
        L2,
        L3,
        L4
    }

    private final DrivetrainSubsystem m_drivetrain;
    private final ElevatorSubsystem m_elevator;
    private final CoralSubsystem m_coral;
    private final AlgaeSubsystem m_algae;
    private final ClimberSubsystem m_climber;

    public Orchestrator(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, CoralSubsystem coral,
            AlgaeSubsystem algae, ClimberSubsystem climber) {
        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_coral = coral;
        m_algae = algae;
        m_climber = climber;
    }

    public Command AimCoral(FieldPosition position, boolean autoAlign) {
        return m_elevator.aim(position).alongWith(m_coral.aim(position));
    }

    public Command IntakeCoral(boolean autoAlign) {
        return AimCoral(FieldPosition.HPSTATION, autoAlign).andThen(m_coral.intake(FieldPosition.HPSTATION));
    }

    public Command ScoreCoral(FieldPosition position, boolean autoAlign, BooleanEvent fire) {
        return AimCoral(position, autoAlign).andThen(m_coral.score(position, fire));
    }

    public Command AimAlgae(FieldPosition position, boolean autoAlign) {
        return m_elevator.aim(position).alongWith(m_algae.aim(position));
    }

    public Command IntakeAlgae(FieldPosition level, boolean autoAlign) {
        if (level != FieldPosition.L2
                && level != FieldPosition.L3) {
            return null;
        }
        return AimAlgae(level, autoAlign).andThen(m_algae.intake(level));
    }

    public Command ScoreAlgae(FieldPosition netOrProcessor, boolean autoAlign, BooleanEvent fire) {
        if (netOrProcessor != FieldPosition.NET
                && netOrProcessor != FieldPosition.PROCESSOR) {
            return null;
        }
        return AimAlgae(netOrProcessor, autoAlign).andThen(m_algae.score(netOrProcessor, fire));
    }

    public Command Climb() {
        return m_climber.aim(FieldPosition.CAGE);
    }

    public Command Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // System.out.println("x: " + xSpeed + ", y: " + ySpeed + ", theta: " + rot);
        return m_drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
}
