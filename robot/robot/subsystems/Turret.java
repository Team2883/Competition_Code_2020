/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Turret extends SubsystemBase 
{
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightSteerCommand = 0.0;
  private static final Command VisionDrive = null;
  final public WPI_TalonSRX m_Turret = new WPI_TalonSRX(Constants.TurretMotor);
  final public WPI_TalonSRX m_Shooter = new WPI_TalonSRX(Constants.ShooterMotor);
  final public WPI_TalonSRX m_kickup = new WPI_TalonSRX(Constants.KickupMotor);
  final public WPI_TalonSRX m_kick = new WPI_TalonSRX(Constants.KickMotor);
  final public WPI_TalonSRX m_agitate = new WPI_TalonSRX(Constants.Agitate);
  private final Solenoid HoodSolenoid = new Solenoid(Constants.HoodSolonoid);
 
  DigitalInput Magencoder = new DigitalInput(Constants.Encoder);
  
  double encodermath;
  double Speed = 1;
  boolean Shooting = false;
  boolean done = false;
  boolean Done = false;
  boolean end = false;
  boolean Kicking = false;
  boolean Kicking1 = false;
  boolean Agitating = false;
  boolean Up = false;

  @Override
  public void periodic() 
  {
    encodermath = (m_Turret.getSelectedSensorPosition() - 1024) / 8;
    SmartDashboard.putNumber("Turret Encoder", encodermath);
  }

  public double getSelectedSensorPosition() 
  {
    return encodermath;
  }

  public void resetEncoder() 
  {
    m_Turret.setSelectedSensorPosition(0);
  }

  public void Autoturn() 
  {
    // These numbers must be tuned for your Robot! Be careful!
    final double TurretMotor = -0.045; // how hard to turn toward the target

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    if (m_LimelightHasValidTarget) 
    {
      m_Turret.set(m_LimelightSteerCommand);
    } 
    else 
    {
      m_Turret.set(Speed);
    }

    // Start with proportional steering
    double steer_cmd = tx * TurretMotor;
    m_LimelightSteerCommand = steer_cmd;
  }

  public void TurretStop() 
  {
    m_Turret.set(0);
  }

  public void ShooterMotorHigh() 
  {
    if (Shooting)
      m_Shooter.set(0);
    else
      m_Shooter.set(-1);

    Shooting = !Shooting;
    done = true;
    isFinished();
  }
  

  public void ShooterMotorLow() 
  {
    if (Shooting)
      m_Shooter.set(0);
    else
      m_Shooter.set(-.85);

    Shooting = !Shooting;
    done = true;
    isFinished();
  }

  public void HoodSolonoid() 
  {
    if (Up)
      HoodSolenoid.set(true);
    else
      HoodSolenoid.set(false);

    Up = !Up;
    done = true;
    isFinished();
  }

  public void Kickup() 
  {
    if (Kicking)
      m_kickup.set(0);
    else
      m_kickup.set(-1);

    Kicking = !Kicking;
    end = true;
    isEnd();
  }
  public void Kicker() 
  {
    if (Kicking1)
      m_kick.set(0);
    else
      m_kick.set(1);

    Kicking1 = !Kicking1;
    done = true;
    isFinished();
  }
  public void Agitate() 
  {
    if (Agitating)
      m_agitate.set(0);
    else
      m_agitate.set(1);

    Agitating = !Agitating;
    done = true;
    isFinished();
  }

  public boolean isEnd()
  {
    return end;
  }
  
  public boolean isDone()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    return Done;
  }

  public void setDefaultCommand() 
  {
    setDefaultCommand(VisionDrive);
  }

  public boolean isFinished() 
  {
    return done;
  }
}
