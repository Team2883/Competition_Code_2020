/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase 
{
  private static final Command Climb = null;
  final public WPI_TalonSRX m_climb = new WPI_TalonSRX(Constants.ClimbMotor);
  private final DoubleSolenoid m_ClimbSolenoid = new DoubleSolenoid(Constants.ClimbSolenoid, Constants.ClimbSolenoid1);
  public boolean done = false;
  public boolean in = false;
  boolean Agitating = false;


  public void Climb(double rotation) 
  {
    m_climb.set(-rotation);
  }
 
 // public void ReverseClimb() 
//  {
 //   m_climb.set(.3);
 // }


 // public void StopClimb() 
 // {
  //  m_climb.set(0);
  //}
  
  public void ClimbSolenoid(boolean in) 
  {
    if (in)
      m_ClimbSolenoid.set(Value.kForward);
    else
      m_ClimbSolenoid.set(Value.kReverse);
  }
public void ClimbReverse(double speed)
{
  m_climb.set(speed);
}

  public boolean isFinished() 
  {
    return done;
  }

  public void setDefaultCommand()
  {
    setDefaultCommand(Climb);
  }
}
