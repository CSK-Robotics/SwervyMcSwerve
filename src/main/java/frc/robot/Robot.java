// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  RobotContainer m_robotContainer = new RobotContainer();

  @Override
  public void autonomousInit() {
    m_robotContainer.configureAutonomous();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.pollControllers();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
