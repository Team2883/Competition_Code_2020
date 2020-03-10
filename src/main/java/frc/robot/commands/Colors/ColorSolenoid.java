// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

//  package frc.robot.commands.Colors;

//  import edu.wpi.first.wpilibj2.command.CommandBase;
//  import frc.robot.subsystems.ColorSolenoidSubsystem;

//  public class ColorSolenoid extends CommandBase 
//  {

//    private final ColorSolenoidSubsystem m_ColorSolenoidSubsystem;

//    public ColorSolenoid(ColorSolenoidSubsystem subsystem) 
//    {
//      m_ColorSolenoidSubsystem = subsystem;

//      addRequirements(m_ColorSolenoidSubsystem);
//    }

//    // Called when the command is initially scheduled.
//    @Override
//    public void initialize() 
//    {  }

//    // Called every time the scheduler runs while the command is scheduled.
//    @Override
//    public void execute() 
//    {
//      m_ColorSolenoidSubsystem.ColorSolenoid(true);
//    }

//    // Called once the command ends or is interrupted.
//    @Override
//    public void end(boolean interrupted) 
//    {
//      m_ColorSolenoidSubsystem.ColorSolenoid(false);
//    }

//    // Returns true when the command should end.
//    @Override
//    public boolean isFinished() 
//    {
//      return false;
//    }
//  }
