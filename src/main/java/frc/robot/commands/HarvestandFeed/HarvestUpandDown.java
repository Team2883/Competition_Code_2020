/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HarvestandFeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;

public class HarvestUpandDown extends CommandBase 
{
  private final Harvester m_harvester;
  boolean done = false;
  boolean UP = false;

  public HarvestUpandDown(Harvester subsystem) 
  {
    m_harvester = subsystem;

    addRequirements(m_harvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
   // done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(UP)
    {
      m_harvester.HarvestSolenoid(false);
    }
    else
    {
      m_harvester.HarvestSolenoid(true);
    }
    UP = !UP;
    done = true;
    isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return done;
  }
}
