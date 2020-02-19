package frc.robot.commands.TurretandShooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;


public class TurretStop extends CommandBase 
{
  private final Turret m_turret;
  
  public TurretStop(Turret subsystem) 
  {
    m_turret = subsystem;
    
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_turret.TurretStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  { 
	return true;
  }
}
