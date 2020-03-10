/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.TurretandShooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;


public class ManualTurret extends CommandBase 
{
  private final Turret m_turret;
  private final DoubleSupplier m_rotation;

  public ManualTurret(Turret subsystem, DoubleSupplier rotation) 
  {
    m_turret = subsystem;
    m_rotation = rotation;
  
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
    m_turret.SetMotor(-m_rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  { 
    m_turret.Turn(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
