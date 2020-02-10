/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.SerialPort;


public class DriveTrain extends SubsystemBase 
{
  // Code Defined Controls or Variables
  private static final Command StickDrive = null;
  private final Solenoid SpeedControl = new Solenoid(Constants.GearShift);
  AHRS ahrs;
  static double sensitivity = 0.8;
  boolean in = false;
  boolean done = false;

  //Encoder Values
  double diameter = 6; // 6 inch sprocket
  double gearRatio = 28.5; //26:1 gearbox
  double distR = (2048 / (diameter * 3.14 / gearRatio) );   // pulse per inch
  double distL = -distR;   // pulse per inch

  //Drivetrain
/**---------------------------------------------------------------------------------------------- */
  //Left Motors
  private final WPI_TalonFX LeftBack = new WPI_TalonFX(Constants.LeftBack);
  private final WPI_TalonFX LeftFront = new WPI_TalonFX(Constants.LeftFront);

  //Right Motors
  private final WPI_TalonFX RightBack = new WPI_TalonFX(Constants.RightBack);
  private final WPI_TalonFX RightFront = new WPI_TalonFX(Constants.RightFront);

  //Grouping
  public SpeedControllerGroup m_Left = new SpeedControllerGroup(LeftBack, LeftFront);
  public SpeedControllerGroup m_Right = new SpeedControllerGroup(RightBack, RightFront);

  //Chassis
  final public DifferentialDrive m_dDrive = new DifferentialDrive(m_Left, m_Right);
/**---------------------------------------------------------------------------------------------- */

  public DifferentialDriveOdometry m_odometry;

  public DriveTrain()
  {
    ahrs = new AHRS(SerialPort.Port.kOnboard);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic()
  {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), LeftBack.getSelectedSensorPosition()/ distL * 0.0254,
    RightBack.getSelectedSensorPosition() / distR * 0.0254);
    
    SmartDashboard.putNumber("Left Falcon Encoder", LeftBack.getSelectedSensorPosition() / distL);
    SmartDashboard.putNumber("Right Falcon Encoder", RightBack.getSelectedSensorPosition() / distR);
    SmartDashboard.putNumber("sensitivity", GetSensitity());
  }
  
  public void Drive(double speed, double rotation)
  {
    m_dDrive.arcadeDrive(speed, rotation * sensitivity);
  }

  public void Shifting()
  {
    if(in)
      SpeedControl.set(false);
    else 
      SpeedControl.set(true);

    in = !in;
    done = true;
    isFinished();
  }

  public boolean isFinished()
  {
    return done;
  }

  //Encoder Methods
/**------------------------------------------------- */
  public double getLeftEncoder()
  {
    return LeftBack.getSelectedSensorPosition();
  }

  public double getRightEncoder()
  {
    return RightBack.getSelectedSensorPosition();
  }

  public void resetEncoder()
  {
    LeftBack.setSelectedSensorPosition(0);
    RightBack.setSelectedSensorPosition(0);
  }

  //Sensitivity Methods
/**------------------------------------------- */
public static void AddSensitivity(double sens)
  {
    sensitivity += sens;
    if(sensitivity > 1)
    {
      sensitivity = 1.0;
    }
    if(sensitivity < .3)
    {
      sensitivity = .3;
    }
  }

  public static double GetSensitity()
  {
    return sensitivity;
  }

  public void sensUP()
  {
    DriveTrain.AddSensitivity(0.1);
  }

  public void sensDN()
  {
    DriveTrain.AddSensitivity(-0.1);
  }

  public void SensReset()
  {
    sensitivity = 0.8;
  }

  
  // Returns the position of the robot on the field.
  public Pose2d getPose() 
  {
    return m_odometry.getPoseMeters();
  }

  // Returns the current wheel speeds of the robot.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() 
  {
    return new DifferentialDriveWheelSpeeds(LeftBack.getSelectedSensorVelocity() / distL * 0.0254, RightBack.getSelectedSensorVelocity() / distR * 0.0254);
  }

  // Resets the odometry to the specified pose.
  public void resetOdometry(Pose2d pose) 
  {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  // Controls the left and right sides of the drive directly with voltages.
  public void tankDriveVolts(double leftVolts, double rightVolts) 
  {
    SmartDashboard.putNumber("PosX", m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("PosY", m_odometry.getPoseMeters().getTranslation().getY());
    m_Left.setVoltage(-leftVolts);
    m_Right.setVoltage(rightVolts);
    m_dDrive.feed();
  }

  // Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() 
  {
    LeftBack.setSelectedSensorPosition(0);
    RightBack.setSelectedSensorPosition(0);
  }

  // Gets the average distance of the two encoders.
  public double getAverageEncoderDistance() 
  {
    return (LeftBack.getSelectedSensorPosition() + RightBack.getSelectedSensorPosition()) / 2.0;
  }

  // the maximum output to which the drive will be constrained
  public void setMaxOutput(double maxOutput) 
  {
    m_dDrive.setMaxOutput(maxOutput);
  }

  // Zeroes the heading of the robot.
  public void zeroHeading() 
  {
    ahrs.reset();
  }

  // Returns the robot's heading in degrees, from -180 to 180
  public double getHeading() 
  {
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  // Returns the turn rate of the robot, in degrees per second
  public double getTurnRate() 
  {
    return ahrs.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setDefaultCommand()
  {
    setDefaultCommand(StickDrive);
  }
}
