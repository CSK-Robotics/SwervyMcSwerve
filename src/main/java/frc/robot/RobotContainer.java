package frc.robot;

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
  private final Orchestrator orchestrator = new Orchestrator(new DrivetrainSubsystem(), new ElevatorSubsystem(),
      new CoralSubsystem(), new AlgaeSubsystem(), new ClimberSubsystem());
  private final DriverControls driver = new DriverControls(orchestrator);
  private final OperatorControls operator = new OperatorControls(orchestrator);

  public RobotContainer() {
    StationData.getInstance(orchestrator);
  }

  public void pollControllers() {
    driver.m_loop.poll();
    operator.m_loop.poll();
  }
}
