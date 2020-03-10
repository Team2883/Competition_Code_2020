/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Drives.AutoDrive;
import frc.robot.commands.HarvestandFeed.Harvest;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class harvestDrive extends ParallelRaceGroup {
  /**
   * Creates a new harvestDrive.
   */
  public harvestDrive(DriveTrain m_drivetrain) {
   addCommands(
    new AutoDrive(40, -1, 0, m_drivetrain));
  }
}
