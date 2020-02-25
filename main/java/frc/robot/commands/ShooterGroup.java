/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.TurretandShooter.ShooterMotorHigh;
import frc.robot.commands.TurretandShooter.TurretVision;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Turret;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterGroup extends SequentialCommandGroup 
{
  private final Turret m_turret;
  private final Harvester m_harvester;

  /**
   * Creates a new ShooterGroup.
   */
  public ShooterGroup(Turret subsystem1, Harvester subsystem2) 
  {
    m_turret = subsystem1;
    m_harvester = subsystem2;
    addRequirements(m_turret, m_harvester);

    addCommands(
      new ShooterMotorHigh(m_turret),
      alongWith(new TurretVision(m_turret)),
      andThen(new WaitCommand(5)),
      andThen(new Kick(m_harvester))  );
  }
}
