// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ElevatorSubsystem.Position;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  //private final XboxController m_controller2 = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final EventLoop m_loop = new EventLoop();
  //private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final Climber m_Climber = new Climber();

  //private final Coral m_coral = new Coral();
  //private final Algae m_algae = new Algae();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  private Command m_autonomousCommand;

  @Override
  public void autonomousInit() {
    m_swerve.setupAutonomousConfigure();
    m_autonomousCommand = getAutonomousCommand();
  }

  @Override
  public void autonomousPeriodic() {
    // m_swerve.updateOdometry();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    m_controller.a(m_loop).ifHigh(m_swerve::reset);
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    //controlElevator();
    controlClimber();
    m_loop.poll();
    //controlCoral();
    // controlAlgae();
  }

  @Override
  public void robotPeriodic() {
    m_swerve.updateOdometry();
    CommandScheduler.getInstance().run();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
        * Drivetrain.kMaxAngularSpeed;
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
    // m_elevator.reachGoal(m_controller.getRightTriggerAxis());
  }
/*
  private void controlElevator() {
    // double elevatorSpeed = MathUtil.applyDeadband(m_controller2.getRightY(),
    // 0.04) * 4;
    // double elevatorSpeed = MathUtil.applyDeadband(m_controller2.getRightY(),
    // 0.04) >= 0.7 ? 12.0 : 0.0;
    // m_elevator.reachGoal(elevatorSpeed);
    if (m_controller2.getStartButton())
      m_elevator.set(Position.ZERO);
    if (m_controller2.getAButton())
      m_elevator.set(Position.L1);
    if (m_controller2.getBButton())
      m_elevator.set(Position.L2);
    if (m_controller2.getXButton())
      m_elevator.set(Position.L3);
    if (m_controller2.getYButton())
      m_elevator.set(Position.L4);
    // SmartDashboard.putNumber("Elevator Setpoint", elevatorSpeed);
  }
 */
  private void controlClimber() {
    double getRightTriggerAxis = m_controller.getRightTriggerAxis();
    double getLeftTriggerAxis = m_controller.getLeftTriggerAxis();
    if (getRightTriggerAxis >= 0.2) {
      getRightTriggerAxis *= 12;
      m_Climber.climb(getRightTriggerAxis);
    } else if (getLeftTriggerAxis >= 0.2) {
      getLeftTriggerAxis *= 12;
      m_Climber.climb(-getLeftTriggerAxis);
    } else {
      m_Climber.climb(0);
    }
  }
/*
  private void controlCoral() {
    double coralSpeed = m_controller2.getLeftY() * 3;
    m_coral.reachGoal(coralSpeed);

    double wheelSpeed = m_controller2.getLeftTriggerAxis() >= 0.2 ? m_controller2.getLeftTriggerAxis()
        : -m_controller2.getRightTriggerAxis();
    m_coral.runWheel(wheelSpeed);
  }

  private void controlAlgae() {
    double wheelSpeed = m_controller.getLeftBumperButton() ? 0.8 : m_controller.getRightBumperButton() ? -0.8 : 0.0;
    //m_algae.runWheel(wheelSpeed);
  }
 */
  public Command getAutonomousCommand() {
    /*
     * try {
     * // Load the path you want to follow using its name in the GUI
     * PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath1");
     * // Create a path following command using AutoBuilder. This will also trigger
     * // event markers.
     * return AutoBuilder.followPath(path);
     * } catch (Exception e) {
     * DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
     * return Commands.none();
     * }
     */

    return m_swerve.run(() -> m_swerve.drive(0.0, 0.5, 0.0, false, getPeriod())).until(() -> isTeleop());
  }
}
