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
import frc.robot.commands.HarvestandFeed.Agitating;
import frc.robot.commands.HarvestandFeed.Harvest;
import frc.robot.commands.HarvestandFeed.HarvestUp;
import frc.robot.commands.HarvestandFeed.HarvestUpandDown;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.TurretandShooter.ShooterMotorHigh;
import frc.robot.commands.TurretandShooter.TurretStop;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FarLeft extends SequentialCommandGroup {
  /**
   * Creates a new Auto2.
   */
  public FarLeft(DriveTrain m_drivetrain, Turret m_turret, Harvester m_harvester, Agitator m_agitator) {
    addCommands(
      new HarvestUpandDown(m_harvester),
      new Harvest(m_harvester),
      new AutoDrive(85, -1, -.1, m_drivetrain),
      new WaitCommand(.25),
      new AutoDrive(1, .1, -.1, m_drivetrain),
      new WaitCommand(.7),
      new HarvestUp(m_harvester),
      new rotateToAngleTimed(1.5, m_drivetrain),
      new AutoDrive(122, -1, -.2, m_drivetrain),
      new AutoDrive(1, .1, 0, m_drivetrain),
      new TurretAuto(m_turret),
      new TurretStop(m_turret),
      new ShooterMotorHigh(m_turret),
      new WaitCommand(1.5),
      new Kick(m_harvester),
      new Agitating(m_agitator),
      new WaitCommand(2.4),
      new Kick(m_harvester),
      new ShooterMotorHigh(m_turret));
  }
}
