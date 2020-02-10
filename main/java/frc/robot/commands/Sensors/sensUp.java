/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Sensors;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class sensUp extends CommandBase 
{
  public final DriveTrain m_driveTrain;
  
  public sensUp(DriveTrain subsystem) 
  {
    m_driveTrain = subsystem;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_driveTrain.sensUP();
  }

  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
