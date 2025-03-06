package frc.robot.HumanInterface;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.EndEffectorSubsystem.FieldPosition;
import frc.robot.Constants.Swerve;
import frc.robot.Commands.Orchestrator;

public class DriverControls {
    private final XboxController m_controller = new XboxController(0);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private boolean m_fieldRelative;

    private final Orchestrator m_orchestrator;
    public final EventLoop m_loop = new EventLoop();
    public final Map<String, Trigger> m_triggers = new HashMap<String, Trigger>();

    public DriverControls(Orchestrator orchestrator) {
        m_orchestrator = orchestrator;
        m_triggers.put("LB: Climb",
                new Trigger(m_controller.leftBumper(m_loop).debounce(0.2)).onTrue(orchestrator.Climb()));
        m_triggers.put("RB: Release Algae",
                new Trigger(m_controller.rightBumper(m_loop).debounce(0.2)).whileTrue(
                        orchestrator.ScoreAlgae(FieldPosition.PROCESSOR, false, m_controller.rightBumper(m_loop))));
        m_controller.start(m_loop).debounce(0.2).ifHigh(() -> m_fieldRelative = !m_fieldRelative);
        m_loop.bind(this::driveWithJoystick);
    }

    public void driveWithJoystick() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
                * Swerve.instanceConstants.driveFreeSpeed();

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
                * Swerve.instanceConstants.driveFreeSpeed();

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
                * Swerve.instanceConstants.driveFreeSpeed();// TODO: #7 calculate maximum angular velocity of robot.

        m_orchestrator.Drive(xSpeed, ySpeed, rot, m_fieldRelative).schedule();
    }
}
