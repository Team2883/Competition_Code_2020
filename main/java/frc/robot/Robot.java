/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Colorings;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Turret;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public static DriveTrain m_driveTrain;
  public static Turret m_turret;
  public static Harvester m_Harvester;
  public static RobotContainer m_robotContainer;
  public static Colorings m_Colorings;
  private Command m_autonomousCommand;
  AHRS ahrs;
  CameraServer server;
  
  @Override
  public void robotInit()
  {
    //m_Harvester.m_kick.set(0);
    m_robotContainer = new RobotContainer(); 
    ahrs = new AHRS(SerialPort.Port.kUSB);
    m_Colorings = new Colorings();
    m_Colorings.setColorTargets();
    ahrs.enableLogging(true);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    server = CameraServer.getInstance();
    server.startAutomaticCapture("cam0",0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */
    SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */
    SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */
    SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() 
  { }

  @Override
  public void disabledPeriodic() 
  { }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    ahrs.zeroYaw();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic()
  {
    CommandScheduler.getInstance().run();
    if (m_driveTrain != null && m_driveTrain.m_odometry != null){
    SmartDashboard.putBoolean("statustrue", m_driveTrain != null);
    SmartDashboard.putNumber("PosX", m_driveTrain.m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("PosY", m_driveTrain.m_odometry.getPoseMeters().getTranslation().getY());
    }
    else {
      SmartDashboard.putBoolean("statusfalse,", m_driveTrain != null);
      if (m_driveTrain != null && m_driveTrain.m_odometry == null){
        m_driveTrain.m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_driveTrain.getHeading()));
      }
    }
  }

  @Override
  public void teleopInit() 
  {
    if (m_autonomousCommand != null) 
    {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      m_Colorings.setColorTargets();
      switch (gameData.charAt(0))
      {
        case 'B' :
          SmartDashboard.putBoolean("Blue Target", true); // Target
          SmartDashboard.putBoolean("Green Target", false);
          SmartDashboard.putBoolean("Red Target", false);
          SmartDashboard.putBoolean("Yellow Target", false);
          break;
        case 'G' :
          SmartDashboard.putBoolean("Blue Target", false);
          SmartDashboard.putBoolean("Green Target", true); // Target
          SmartDashboard.putBoolean("Red Target", false);
          SmartDashboard.putBoolean("Yellow Target", false);
          break;
        case 'R' :
          SmartDashboard.putBoolean("Blue Target", false);
          SmartDashboard.putBoolean("Green Target", false);
          SmartDashboard.putBoolean("Red Target", true); // Target
          SmartDashboard.putBoolean("Yellow Target", false);
          break;
        case 'Y' :
          SmartDashboard.putBoolean("Blue Target", false);
          SmartDashboard.putBoolean("Green Target", false);
          SmartDashboard.putBoolean("Red Target", false);
          SmartDashboard.putBoolean("Yellow Target", true); // Target
          break;
      }
    }
    else
    {
      //No Target Yet
      SmartDashboard.putBoolean("Target Found", false);
      SmartDashboard.putBoolean("Blue Target", false);
      SmartDashboard.putBoolean("Green Target", false);
      SmartDashboard.putBoolean("Red Target", false);
      SmartDashboard.putBoolean("Yellow Target", false);
    }
  }

  @Override
  public void testInit() 
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  { }
}
