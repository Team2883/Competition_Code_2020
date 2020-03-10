/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class rotateToAngleTimed extends ParallelRaceGroup {
  /**
   * Creates a new rotateToAngleTimed.
   */
  public rotateToAngleTimed(double time, DriveTrain m_driveTrain) 
  {
    addCommands(
      new RotateToAngle(-115, 1, .65, m_driveTrain),
      new WaitCommand(time));
  }
}
