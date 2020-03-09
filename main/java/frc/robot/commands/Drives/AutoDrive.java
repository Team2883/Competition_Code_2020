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
  private final DriveTrain m_driveTrain;
  private final double m_distance;
  private final double m_speed;
  private final double m_rotation;
  double diameter = 6; // 6 inch wheel
  double gearRatio = 28.5; // 26:1 gearbox
  double distR = (2048 / (diameter * 3.14 / gearRatio) );
  
  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   * @param rotation
   */
  public AutoDrive(double inches, double speed, double rotation, DriveTrain subsystem) 
  {
    m_distance = inches;
    m_speed = speed;
    m_rotation = rotation;
    m_driveTrain = subsystem;

    addRequirements(m_driveTrain);
  }

  @Override
  public void initialize() 
  {
    m_driveTrain.resetEncoders();
    m_driveTrain.Drive(0, 0);
  }

  @Override
  public void execute()
  {
    m_driveTrain.Drive(m_speed, m_rotation);
  }

  @Override
  public void end(boolean interrupted) 
  {
    m_driveTrain.Drive(0, 0);
  }

  @Override
  public boolean isFinished() 
  {
    if(m_distance > 0)
      return Math.abs(m_driveTrain.getRightEncoder() / distR) >= m_distance;
    else if(m_distance < 0)
      return Math.abs(m_driveTrain.getRightEncoder()/ distR) *-1 <= m_distance;
    else 
      return false;
  }
}
