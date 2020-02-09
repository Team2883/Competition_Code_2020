package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AutoTurret extends CommandBase 
{
  private final Turret m_turret;

  /**
   * Creates a new ShooterMotorHigh.
   */
  public AutoTurret(Turret subsystem) 
  {
    m_turret = subsystem;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {  
     m_turret.Autoturn();
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
