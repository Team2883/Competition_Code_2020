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
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Turret extends SubsystemBase 
{
  private static final Command VisionDrive = null;
  final public WPI_TalonSRX m_Turret = new WPI_TalonSRX(Constants.TurretMotor);
  final public WPI_TalonSRX m_Shooter1 = new WPI_TalonSRX(Constants.ShooterMotor);
  final public WPI_TalonSRX m_Shooter2 = new WPI_TalonSRX(Constants.ShooterMotor2);
  final public WPI_TalonSRX m_topKick = new WPI_TalonSRX(Constants.TopKickMotor);
  public final SpeedControllerGroup m_Shooter = new SpeedControllerGroup(m_Shooter1, m_Shooter2);
  private final Solenoid HoodSolenoid = new Solenoid(Constants.HoodSolenoid);
  DigitalInput Magencoder = new DigitalInput(Constants.Encoder);
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightSteerCommand = 0.0;
  double encodermath;
  double Speed = 1;
  boolean Shooting = false;
  boolean done = false;
  boolean Done = false;
  boolean end = false;
  boolean topKick = false;
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

  public void HoodSolenoid() 
  {
    if (Up)
      HoodSolenoid.set(true);
    else
      HoodSolenoid.set(false);

    Up = !Up;
    done = true;
    isFinished();
  }

  public void topKick() 
  {
    if (topKick)
      m_topKick.set(0);
    else
      m_topKick.set(-1);

    topKick = !topKick;
    end = true;
    isEnd();
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

  public void TurretStop() 
  {
    m_Turret.set(0);
  }

  public boolean isDone()
  {
    
    return Done;
  }
  
  public boolean isEnd()
  {
    return end;
  }

  public boolean isFinished() 
  {
    return done;
  }

  public void setDefaultCommand() 
  {
    setDefaultCommand(VisionDrive);
  }
}
