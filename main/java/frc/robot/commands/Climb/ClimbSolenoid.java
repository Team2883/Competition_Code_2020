package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;


public class ClimbSolenoid extends CommandBase 
{
  private final Climber m_climber;
  boolean done = false;
  boolean UP = false;
  
  public ClimbSolenoid(Climber subsystem) 
  {
    m_climber = subsystem;
  
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  { 
    if(UP)
    {
      m_climber.ClimbSolenoid(false);
    }
    else
    {
      m_climber.ClimbSolenoid(true);
    }
    UP = !UP;
    done = true;
    isFinished();  
  }

  @Override
  public void end(boolean interupted){
      
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    
    return done;
  }
}