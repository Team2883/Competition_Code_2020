package frc.robot.commands.HarvestandFeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;


public class HarvesterReverse extends CommandBase
{
    private final Harvester m_Harvester;

    public HarvesterReverse(Harvester subsystem)
    {
        m_Harvester = subsystem;
        
        addRequirements(m_Harvester);
    }

    @Override
    public void initialize()
    {
        m_Harvester.Harvest(-0.45);
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