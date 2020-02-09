/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drives;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Shift extends CommandBase
{
    private final DriveTrain m_DriveTrain;

    public Shift(DriveTrain subsystem)
    {
        m_DriveTrain = subsystem;
        addRequirements(m_DriveTrain);
    }

    @Override
    public void initialize()
    {
        m_DriveTrain.Shifting();
    }
    
    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
