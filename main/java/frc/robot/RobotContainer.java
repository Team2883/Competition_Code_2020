/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos.Auto1;
import frc.robot.commands.Climb.Climb;
import frc.robot.commands.Climb.ClimbReverse;
import frc.robot.commands.Climb.ClimbSolenoid;
import frc.robot.commands.Climb.ClimbStop;
import frc.robot.commands.Colors.Colored;
import frc.robot.commands.Drives.AutoDrive;
import frc.robot.commands.Drives.Shift;
import frc.robot.commands.Drives.StickDrive;
import frc.robot.commands.HarvestandFeed.Agitating;
import frc.robot.commands.HarvestandFeed.AgitatorStop;
import frc.robot.commands.HarvestandFeed.Harvest;
import frc.robot.commands.HarvestandFeed.HarvestStop;
import frc.robot.commands.HarvestandFeed.HarvestUpandDown;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.HarvestandFeed.ReverseAgitate;
import frc.robot.commands.Sensors.sensDown;
import frc.robot.commands.Sensors.sensUp;
import frc.robot.commands.TurretandShooter.ManualTurret;
import frc.robot.commands.TurretandShooter.ShooterMotorHigh;
import frc.robot.commands.TurretandShooter.TurretSolenoid;
import frc.robot.commands.TurretandShooter.TurretStop;
import frc.robot.commands.TurretandShooter.TurretVision;
import frc.robot.commands.TurretandShooter.hoodSolenoid;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Colorings;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Turret;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  public final XboxController stick = new XboxController(Constants.XboxPort);
  public final XboxController stick1 = new XboxController(Constants.XboxPort1);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Turret m_turret = new Turret();
  private final Colorings m_Colorings = new Colorings();
  private final Harvester m_harvester = new Harvester();
  private final Climber m_climber = new Climber();
  private final Agitator m_agitator = new Agitator();
 

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    final Command m_simpleAuto = new AutoDrive(0, 0, m_driveTrain);
    final Command m_complexCommand = new Auto1(m_driveTrain, m_turret, m_harvester);

    // Configure the button bindings
    configureButtonBindings();
  
    //Creates Left Stick On Controller
/**------------------------------------------------- */
    m_driveTrain.setDefaultCommand(
     new StickDrive(
       m_driveTrain,
       () -> stick.getY(GenericHID.Hand.kLeft), 
       () -> -stick.getX(GenericHID.Hand.kLeft)));
/**------------------------------------------------- */
    m_climber.setDefaultCommand(
      new Climb(
       m_climber,
       () -> stick.getRawAxis(3))); 

/********************************************* */
    m_turret.setDefaultCommand(
      new ManualTurret(
        m_turret,
        () -> stick1.getX(GenericHID.Hand.kLeft)));

    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexCommand);
    SmartDashboard.putData("Autonomous", m_chooser);
  }
  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    final JoystickButton ClimbReverseButton = new JoystickButton(stick, Constants.ClimbReverseButton);
    final JoystickButton ClimbSolenoidButton = new JoystickButton(stick, Constants.ClimbSolenoidButton);
    final JoystickButton HarvestButton = new JoystickButton(stick, Constants.Harvest);
    final JoystickButton HarvestSolenoidButton = new JoystickButton(stick, Constants.HarvestSolenoid);
    final JoystickButton HoodSolenoidButton = new JoystickButton(stick1, Constants.Hoodsolenoid);
    final JoystickButton ShifterButton = new JoystickButton(stick, Constants.ShifterButton);
    final JoystickButton TurretSolenoidButton = new JoystickButton(stick1, Constants.TurretSolenoidButton);
    
    final JoystickButton AgitatorButton = new JoystickButton(stick1, Constants.Agitator);
    final JoystickButton AutoTurnButton = new JoystickButton(stick1, Constants.AutoTurnButton);
    final JoystickButton Colored = new JoystickButton(stick1, Constants.Colored);
    final JoystickButton HighShootButton = new JoystickButton(stick1, Constants.HighShootButton);
    final JoystickButton kickbutton = new JoystickButton(stick1, Constants.KickButton);
    final JoystickButton LowerSensButton = new JoystickButton(stick1, Constants.LowerSensButton);
    //final JoystickButton LowShootButton = new JoystickButton(stick1, Constants.LowShootButton);
    final JoystickButton RaiseSensButton = new JoystickButton(stick1, Constants.RaiseSensButton);
    final JoystickButton ReverseAgitate = new JoystickButton(stick1, Constants.ReverseAgitate);


    ClimbReverseButton.whileHeld(new ClimbReverse(m_climber));
    ClimbReverseButton.whenReleased(new ClimbStop(m_climber));
    ClimbSolenoidButton.whenPressed(new ClimbSolenoid(m_climber));
    HarvestButton.whenPressed(new Harvest (m_harvester));
    HarvestButton.whenReleased(new HarvestStop (m_harvester));
    HarvestSolenoidButton.whenPressed(new HarvestUpandDown(m_harvester));
    HoodSolenoidButton.whenPressed(new hoodSolenoid(m_turret));
    ShifterButton.whenPressed(new Shift(m_driveTrain));
    TurretSolenoidButton.whenPressed(new TurretSolenoid(m_turret));

    AgitatorButton.whileHeld(new Agitating(m_agitator)); 
    AgitatorButton.whenReleased(new AgitatorStop(m_agitator));
    AutoTurnButton.whileHeld(new TurretVision(m_turret));
    AutoTurnButton.whenReleased(new TurretStop(m_turret));
    Colored.whileHeld(new Colored(m_Colorings));
    HighShootButton.whenPressed(new ShooterMotorHigh(m_turret));
    kickbutton.whenPressed(new Kick(m_harvester));
    LowerSensButton.whenPressed(new sensDown(m_driveTrain));
    //LowShootButton.whenPressed(new ShooterMotorLow(m_turret));
    RaiseSensButton.whenPressed(new sensUp(m_driveTrain));
    ReverseAgitate.whileHeld(new ReverseAgitate(m_agitator));
    ReverseAgitate.whenReleased(new AgitatorStop(m_agitator));
  }

  /**
   * Use this to pass the autoDrivenomous command to the main {@link Robot} class.
   *
   *  the command to run in autoDrivenomous
   */
  public Command getAutonomousCommand() 
  {
    return m_chooser.getSelected();
  }

  public XboxController getController() 
  {
    return stick;
  }

  public XboxController getController1() 
  {
    return stick1;
  }
}
