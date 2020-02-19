/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drives.AutoDrive;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.TurretandShooter.ShooterMotorHigh;
import frc.robot.commands.TurretandShooter.TurretStop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Harvester;


/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class Auto1 extends SequentialCommandGroup 
{
  /**
   * @param Drive The Drive subsystem this command will run on
   * @param Turret The Turret subsystem this command will run on
   * @param Harvester The Harvester subsystem this command will run on
   */
  public Auto1(DriveTrain m_driveTrain, Turret m_turret, Harvester m_harvester) 
  {
    addCommands(
    new TurretAuto(m_turret),
    new ShooterMotorHigh(m_turret),
    new WaitCommand(.5),
    new Kick(m_harvester),
    new WaitCommand(2),

    new Kick(m_harvester),
    new ShooterMotorHigh(m_turret),
    new TurretStop(m_turret),
 
    new AutoDrive(30, 1, m_driveTrain));
    }
}
