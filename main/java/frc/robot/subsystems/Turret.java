/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
  private static final Command VisionDrive = null;
  final public WPI_TalonSRX m_Turret = new WPI_TalonSRX(Constants.TurretMotor);
  final public WPI_TalonSRX m_topKick = new WPI_TalonSRX(Constants.TopKickMotor);
  private final Solenoid HoodSolenoid = new Solenoid(Constants.HoodSolenoid);
  private final Solenoid TurretSolenoid = new Solenoid(Constants.TurretSolenoid);
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightSteerCommand = 0.0;
  DigitalInput LeftSwitch = new DigitalInput(Constants.LeftSwitch);
  DigitalInput RightSwitch = new DigitalInput(Constants.RightSwitch);
  double encodermath;
  double Speed = 1;
  boolean Shooting = false;
  boolean done = false;
  boolean Done = false;
  boolean end = false;
  boolean topKick = false;
  boolean Up = false;
  boolean out = false;

  @Override
  public void periodic() 
  {
    encodermath = (m_Turret.getSelectedSensorPosition() - 1024) / 8;
    SmartDashboard.putNumber("Turret Encoder", encodermath);
    SmartDashboard.putBoolean("RightSwitch", RightSwitch.get());
    SmartDashboard.putBoolean("LeftSwitch", LeftSwitch.get());
    SmartDashboard.putNumber("TurretValue", m_Turret.get());
  }

  public double getSelectedSensorPosition() 
  {
    return encodermath;
  }

  public void resetEncoder() 
  {
    m_Turret.setSelectedSensorPosition(0);
  }

  public void Turn(double rotation)
  {
    SetMotor(rotation);
  }
 
  public void Autoturn() 
  {
    // These numbers must be tuned for your Robot! Be careful!
    final double TurretMotor = -0.045; // how hard to turn toward the target

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    
    SmartDashboard.putNumber("limelightValue", tx);
    if (tv < 1.0) 
    {
      m_LimelightHasValidTarget = false;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    if (m_LimelightHasValidTarget) 
    {
        Speed = m_LimelightSteerCommand;
    } 
    else 
    {
      Speed = 0;
    }

    SetMotor(Speed);
    // Start with proportional steering
    double steer_cmd = tx * TurretMotor;
    m_LimelightSteerCommand = steer_cmd;
  }

  public void SetMotor(double TurretSpeed) 
  {

  if(TurretSpeed > 0 && LeftSwitch.get())
     m_Turret.set(TurretSpeed);
  
  
  else if(TurretSpeed < 0 && RightSwitch.get()) 
     m_Turret.set(TurretSpeed);

    else 
      m_Turret.set(0);

  }

  public void HoodSolenoid() 
  {
    if (Up)
      TurretSolenoid.set(true);
    else
      TurretSolenoid.set(false);

    Up = !Up;
    done = true;
    isFinished();
  }

  public void TurretSolenoid() 
  {
    if (out)
      HoodSolenoid.set(true);
    else
      HoodSolenoid.set(false);

    out = !out;
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

  public void TurretStop() 
  {
    m_Turret.set(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
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
