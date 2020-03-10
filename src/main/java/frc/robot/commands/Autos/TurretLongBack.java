/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.TurretandShooter.TurretVision;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;


public class TurretLongBack extends ParallelRaceGroup 
{
  /**
   * Creates a new TurretLongBack.
   */
  public TurretLongBack(Turret m_turret, DriveTrain m_drivetrain) 
  {
    addCommands(
      // new TurretVision(m_turret),
      new RotateToAngle(-57, 1, .34, m_drivetrain));
  }
}
