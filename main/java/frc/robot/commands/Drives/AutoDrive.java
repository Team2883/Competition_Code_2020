/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drives;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class AutoDrive extends CommandBase 
{
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  double driveSpeed = 0;
  double driveRotation = 0;
  
  public AutoDrive(double speed, double rotation) 
  {
    driveSpeed = speed;
    driveRotation = rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    Robot.m_driveTrain.m_dDrive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
