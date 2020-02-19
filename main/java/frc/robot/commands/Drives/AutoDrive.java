/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drives;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class AutoDrive extends CommandBase 
{
  private final DriveTrain m_drive;
  private final double m_distance;
  private final double m_speed;
  double diameter = 6; // 6 inch wheel
  double gearRatio = 28.5; // 26:1 gearbox
  double distR = (2048 / (diameter * 3.14 / gearRatio) );
  
  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public AutoDrive(double inches, double speed, DriveTrain drive) 
  {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
  }

  @Override
  public void initialize() 
  {
    m_drive.resetEncoders();
    m_drive.DriveAuto(0, 0);
  }

  @Override
  public void execute()
  {
    m_drive.DriveAuto(m_speed, 0);
  }

  @Override
  public void end(boolean interrupted) 
  {
    m_drive.DriveAuto(0, 0);
  }

  @Override
  public boolean isFinished() 
  {
    if(m_distance > 0)
      return Math.abs(m_drive.getRightEncoder() / distR) >= m_distance;
    else if(m_distance < 0)
      return Math.abs(m_drive.getRightEncoder()/ distR) <= m_distance;
    else 
      return false;
  }
}
