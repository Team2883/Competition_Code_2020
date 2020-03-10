/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TurretandShooter.TurretVision;
import frc.robot.subsystems.Turret;


public class TurretAuto2 extends ParallelRaceGroup 
{
  /**
   * @param Turret
   */
  public TurretAuto2(double time, Turret m_turret) 
  {
    addCommands(
      new TurretVision(m_turret),
      new WaitCommand(time));
  }
}
