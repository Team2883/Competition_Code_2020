package frc.robot.commands.HarvestandFeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;


public class HarvestStop extends CommandBase
{
    private final Harvester m_Harvester;

    public HarvestStop(Harvester subsystem)
    {
        m_Harvester = subsystem;
        
        addRequirements(m_Harvester);
    }

    @Override
    public void initialize()
    {
        m_Harvester.HarvestStop();
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
