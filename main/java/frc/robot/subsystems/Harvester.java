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


public class Harvester extends SubsystemBase 
{
  private static final Command Harvest = null;
  final public WPI_TalonSRX m_Harvester = new WPI_TalonSRX(Constants.HarvesterMotor);
  final public WPI_TalonSRX m_kick = new WPI_TalonSRX(Constants.BottomKickMotor);
  final public WPI_TalonSRX m_agitate = new WPI_TalonSRX(Constants.Agitate);
  private final DoubleSolenoid HarvestSoloenoid = new DoubleSolenoid(Constants.Harvestsolenoid1, Constants.Harvestsolenoid2);
  public boolean done = false;
  public boolean  Harvesting = false;
  public boolean  In = false;
  boolean Done = false;
  boolean bottomKick = false;
  boolean Agitating = false;

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

  public void Harvest() 
  {
    if (Harvesting)
      m_Harvester.set(0);
    else
      m_Harvester.set(-.45);

    Harvesting = !Harvesting;
    done = true;
    isFinished();
  }

  public void HarvestSolenoid(boolean Raise) 
  {
    if (Raise)
      HarvestSoloenoid.set(Value.kReverse);
    else
      HarvestSoloenoid.set(Value.kForward);
  }
  
  public void bottomKick() 
  {
    if (bottomKick)
      m_kick.set(0);
    else
      m_kick.set(1);

    bottomKick = !bottomKick;
    done = true;
    isFinished();
  }

  public boolean isFinished() 
  {
    return done;
  }

  public void setDefaultCommand()
  {
    setDefaultCommand(Harvest);
  }
}
