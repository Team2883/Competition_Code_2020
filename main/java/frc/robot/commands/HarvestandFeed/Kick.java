/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HarvestandFeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;


public class Kick extends CommandBase 
{
  private final Harvester m_harvester;

  public Kick(Harvester subsystem) 
  {
    m_harvester = subsystem;

    addRequirements(m_harvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_harvester.bottomKick();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { }

   // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
