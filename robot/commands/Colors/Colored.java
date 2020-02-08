/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Colors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Colorings;

public class Colored extends CommandBase 
{
  private final Colorings m_Colorings;
  
  public Colored(Colorings subsystem) 
  {
    m_Colorings = subsystem;

    addRequirements(m_Colorings);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();

    switch (gameData.charAt(0))
    {
      case 'B':
        m_Colorings.Blue();
        break;
      case 'G':
        m_Colorings.Green();
        break;
      case 'R':
        m_Colorings.Red();
        break;
      case 'Y':
        m_Colorings.Yellow();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
