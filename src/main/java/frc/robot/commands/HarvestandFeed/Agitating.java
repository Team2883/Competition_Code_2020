package frc.robot.commands.HarvestandFeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Agitator;


public class Agitating extends CommandBase 
{
  private final Agitator m_agitator;
  
  public Agitating(Agitator subsystem) 
  {
    m_agitator = subsystem;
  
    addRequirements(m_agitator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
   m_agitator.Agitate(-1);
  }

   // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
