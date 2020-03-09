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
import frc.robot.commands.HarvestandFeed.HarvestUpandDown;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.TurretandShooter.ShooterMotorHigh;
import frc.robot.commands.TurretandShooter.TurretStop;
import frc.robot.commands.TurretandShooter.TurretVision;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MiddleShoot extends SequentialCommandGroup {
  /**
   * Creates a new MiddleShoot.
   */
  public MiddleShoot(DriveTrain m_drivetrain, Turret m_turret, Harvester m_harvester) {
    addCommands(
      new TurretAuto(m_turret),
      new ShooterMotorHigh(m_turret),
      new WaitCommand(3),
      new Kick(m_harvester),
      
      new WaitCommand(3.5),
      new Kick(m_harvester),
      new ShooterMotorHigh(m_turret),
      new TurretStop(m_turret),
      new HarvestUpandDown(m_harvester),
      //new HarvestUpandDown(m_harvester),
      //new Harvest(m_harvester),
      new AutoDrive(65, 1, .2, m_drivetrain));
      // new TurretLongBack(m_turret, m_drivetrain),
      // new WaitCommand(1),
      // new Harvest(m_harvester),
      // new AutoDrive(40, -0.8, 0, m_drivetrain),
      // new TurretAuto(m_turret),
      // new ShooterMotorHigh(m_turret),
      // new WaitCommand(1),
      // new Kick(m_harvester),
      // new Kick(m_harvester),
      // new WaitCommand(.2),
      // new Kick(m_harvester));
      
   
  }
}
