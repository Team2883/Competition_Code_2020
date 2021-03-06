package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;


public class ClimbReverse extends CommandBase
{
    private final Climber m_climber;

    public ClimbReverse(Climber subsystem)
    {
        m_climber = subsystem;

        addRequirements(m_climber);
    }

    @Override
    public void initialize()
    {
       
    }

    @Override
    public void execute() {
        m_climber.m_climb.set(1);
    }
    
    @Override
    public boolean isFinished() 
    {
        
        return true;
    }
}
