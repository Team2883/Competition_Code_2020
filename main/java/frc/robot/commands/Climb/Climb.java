/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climb;

import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Climb extends CommandBase 
{
  private final Climber m_climber;
  private final DoubleSupplier m_rotation;
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  public Climb(Climber subsystem, DoubleSupplier rotation) 
  {
    m_climber = subsystem;
    m_rotation = rotation;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_climber.Climb(m_rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_climber.m_climb.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}