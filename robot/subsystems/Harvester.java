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


public class Harvester extends SubsystemBase 
{
  private static final Command Harvest = null;
  final public WPI_TalonSRX m_Harvester = new WPI_TalonSRX(Constants.HarvesterMotor);
  public boolean done = false;
  public boolean  Harvesting = false;

  public void Harvest() 
  {
    if (Harvesting)
      m_Harvester.set(0);
    else
      m_Harvester.set(-1);

    Harvesting = !Harvesting;
    done = true;
    isFinished();
  }
  
  public void setDefaultCommand()
  {
    setDefaultCommand(Harvest);
  }

  public boolean isFinished() 
  {
    return done;
  }
}
