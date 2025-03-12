package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Orchestrator;
import frc.robot.HumanInterface.DriverControls;
import frc.robot.HumanInterface.OperatorControls;
import frc.robot.HumanInterface.StationData;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem m_swerve = new DrivetrainSubsystem();
  /*
   * private final Orchestrator orchestrator = new Orchestrator(m_swerve, new
   * ElevatorSubsystem(),
   * new CoralSubsystem(), new AlgaeSubsystem(), new ClimberSubsystem());
   */
  private final Orchestrator orchestrator = new Orchestrator(m_swerve, null, null, null, null);
  private final DriverControls driver = new DriverControls(orchestrator);
  //private final OperatorControls operator = new OperatorControls(orchestrator);

  public RobotContainer() {
    StationData.getInstance(orchestrator);
  }

  public void pollControllers() {
    driver.m_loop.poll();
    //driver.driveWithJoystick().schedule();
    //operator.m_loop.poll();
  }

  public void configureAutonomous() {
    m_swerve.setupAutonomousConfigure();

    var m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  public Command getAutonomousCommand() {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath1");
      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
