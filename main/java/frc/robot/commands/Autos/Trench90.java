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
import frc.robot.commands.HarvestandFeed.Harvest;
import frc.robot.commands.HarvestandFeed.HarvestStop;
import frc.robot.commands.HarvestandFeed.HarvestUpandDown;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.TurretandShooter.ShooterMotorHigh;
import frc.robot.commands.TurretandShooter.TurretSolenoid;
import frc.robot.commands.TurretandShooter.TurretStop;
import frc.robot.commands.TurretandShooter.hoodSolenoid;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Harvester;


/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class Trench90 extends SequentialCommandGroup 
{
  /**
   * @param Drive The Drive subsystem this command will run on
   * @param Turret The Turret subsystem this command will run on
   * @param Harvester The Harvester subsystem this command will run on
   */
  public Trench90(DriveTrain m_driveTrain, Turret m_turret, Harvester m_harvester) 
  {
    addCommands(

    new TurretAuto(m_turret),
    new TurretStop(m_turret),
    new ShooterMotorHigh(m_turret),
    new WaitCommand(1),
    new Kick(m_harvester),
    new WaitCommand(2.4),
    new Kick(m_harvester),
    new ShooterMotorHigh(m_turret),
    new TurretStop(m_turret),

    //new RotateToAngle(-84, 1, .34, m_driveTrain),
    new HarvestUpandDown(m_harvester),
    new Harvest(m_harvester),
    new hoodSolenoid(m_turret),
    new TurretSolenoid(m_turret),
    new AutoDrive(200, -1, -.1, m_driveTrain),
    new HarvestUpandDown(m_harvester),
    //new TurretLongBack(m_turret, m_driveTrain),
    new HarvestStop(m_harvester),
    new TurretStop(m_turret));
    //new AutoDrive(25, 1, 0, m_driveTrain),
    // new TurretAuto(m_turret));
    }
}
