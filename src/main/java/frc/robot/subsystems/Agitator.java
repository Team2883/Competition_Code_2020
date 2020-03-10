/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Agitator extends SubsystemBase 
{
  private static final Command Agitating = null;
  final public WPI_TalonSRX m_agitate = new WPI_TalonSRX(Constants.Agitate);


  public void Agitate(double speed) 
  {
   m_agitate.set(speed);
  }
  
  public void AgitateStop(double speed) 
  {
   m_agitate.set(0);
  }

  public void setDefaultCommand()
  {
    setDefaultCommand(Agitating);
  }
}
