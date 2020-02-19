/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HarvestandFeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;


public class Harvest extends CommandBase
{
    private final Harvester m_Harvester;

    public Harvest(Harvester subsystem)
    {
        m_Harvester = subsystem;
        
        addRequirements(m_Harvester);
    }

    @Override
    public void initialize()
    {
        m_Harvester.Harvest(0.45);
    }
    @Override
    public void execute() 
    {
        
     }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
