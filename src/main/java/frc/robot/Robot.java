// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Commands.Orchestrator;
import frc.robot.HumanInterface.DriverControls;
import frc.robot.HumanInterface.OperatorControls;
import frc.robot.Subsystems.AlgaeSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;

public class Robot extends TimedRobot {
  private final Orchestrator orchestrator = new Orchestrator(new DrivetrainSubsystem(), new ElevatorSubsystem(),
      new CoralSubsystem(), new AlgaeSubsystem(), new ClimberSubsystem());
  private final DriverControls driver = new DriverControls(orchestrator);
  private final OperatorControls operator = new OperatorControls(orchestrator);

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    driver.m_loop.poll();
    operator.m_loop.poll();
  }

  @Override
  public void robotPeriodic() {
  }
}
