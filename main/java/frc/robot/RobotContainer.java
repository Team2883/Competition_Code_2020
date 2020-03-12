/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos.FarLeft;
import frc.robot.commands.Autos.MiddleShoot;
import frc.robot.commands.Autos.SimpleAuto;
import frc.robot.commands.Autos.Trench90;
import frc.robot.commands.Climb.Climb;
import frc.robot.commands.Climb.ClimbReverse;
import frc.robot.commands.Climb.ClimbSolenoid;
import frc.robot.commands.Climb.ClimbStop;
// import frc.robot.commands.Colors.ColorSolenoid;
// import frc.robot.commands.Colors.ColorWheel;
import frc.robot.commands.Drives.Shift;
import frc.robot.commands.Drives.StickDrive;
import frc.robot.commands.HarvestandFeed.Agitating;
import frc.robot.commands.HarvestandFeed.AgitatorStop;
import frc.robot.commands.HarvestandFeed.Harvest;
import frc.robot.commands.HarvestandFeed.HarvestStop;
import frc.robot.commands.HarvestandFeed.HarvestUpandDown;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.HarvestandFeed.ReverseAgitate;
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
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  public boolean isShooterPressed = false;
  public final XboxController stick = new XboxController(Constants.XboxPort);
  public final XboxController stick1 = new XboxController(Constants.XboxPort1);
  final JoystickButton PIDShootButton = new JoystickButton(stick1, Constants.PIDShootButton);
  private boolean lastShooterVal = false;
  public boolean Toggler = false;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final Agitator m_agitator = new Agitator();
  private final Climber m_climber = new Climber();
  //  private final ColorSolenoidSubsystem m_colorSolenoidSubsystem = new ColorSolenoidSubsystem();
  //  private final Colorings m_colorings = new Colorings();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Harvester m_harvester = new Harvester();
  private final Turret m_turret = new Turret();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
 
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    final Command m_FarLeft = new FarLeft(m_driveTrain, m_turret, m_harvester, m_agitator);
    final Command m_MiddleShoot = new MiddleShoot(m_driveTrain, m_turret, m_harvester);
    final Command m_SimpleAuto = new SimpleAuto(m_harvester, m_turret, m_driveTrain);
    final Command m_Trench90 = new Trench90(m_driveTrain, m_turret, m_harvester);
    
    m_chooser.addOption("Far Left", m_FarLeft);
    m_chooser.setDefaultOption("MiddleShoot", m_MiddleShoot);
     m_chooser.addOption("Simple Auto", m_SimpleAuto);
     m_chooser.addOption("Trench 90", m_Trench90);
    SmartDashboard.putData("Autonomous", m_chooser);

    //Creates Left Stick On 1st Controller
/**------------------------------------------------- */
    m_driveTrain.setDefaultCommand(
     new StickDrive(
        m_driveTrain,
        () -> stick.getY(GenericHID.Hand.kLeft), 
        () -> -stick.getX(GenericHID.Hand.kLeft)));

    //Creates Right Stick On 1st Controller
/**------------------------------------------------- */
    m_climber.setDefaultCommand(
      new Climb(
        m_climber,
        () -> stick.getRawAxis(3))); 

    //Creates Left Stick On 2nd Controller
/********************************************* */
    m_turret.setDefaultCommand(
      new ManualTurret(
        m_turret,
        () -> stick1.getX(GenericHID.Hand.kLeft)));
    
    configureButtonBindings();
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
    final JoystickButton Colored = new JoystickButton(stick, Constants.Colored);
    final JoystickButton ColorsSolenoid = new JoystickButton(stick, Constants.ColorsSolenoid);
    final JoystickButton HarvestButton = new JoystickButton(stick, Constants.Harvest);
    final JoystickButton HarvestSolenoidButton = new JoystickButton(stick, Constants.HarvestSolenoid);
    final JoystickButton ShifterButton = new JoystickButton(stick, Constants.ShifterButton);

    final JoystickButton AgitatorButton = new JoystickButton(stick1, Constants.Agitator);
    final JoystickButton AutoTurnButton = new JoystickButton(stick1, Constants.AutoTurnButton);
    final JoystickButton HighShootButton = new JoystickButton(stick1, Constants.HighShootButton);
    final JoystickButton HoodSolenoidButton = new JoystickButton(stick1, Constants.Hoodsolenoid);
    final JoystickButton kickbutton = new JoystickButton(stick1, Constants.KickButton);
    final JoystickButton ReverseAgitate = new JoystickButton(stick1, Constants.ReverseAgitate);
    final JoystickButton TurretSolenoidButton = new JoystickButton(stick1, Constants.TurretSolenoidButton);
    


    ClimbReverseButton.whileHeld(new ClimbReverse(m_climber));
    ClimbReverseButton.whenReleased(new ClimbStop(m_climber));
    ClimbSolenoidButton.whenPressed(new ClimbSolenoid(m_climber));
    // Colored.whileHeld(new ColorWheel(m_colorings));
    // ColorsSolenoid.whileHeld(new ColorSolenoid(m_colorSolenoidSubsystem));
    HarvestButton.whileHeld(new Harvest(m_harvester));
    HarvestButton.whenReleased(new HarvestStop (m_harvester));
    HarvestSolenoidButton.whenPressed(new HarvestUpandDown(m_harvester));
    ShifterButton.whenPressed(new Shift(m_driveTrain));

    AgitatorButton.whileHeld(new Agitating(m_agitator)); 
    AgitatorButton.whenReleased(new AgitatorStop(m_agitator));
    AutoTurnButton.whileHeld(new TurretVision(m_turret));
    AutoTurnButton.whenReleased(new TurretStop(m_turret));
    HighShootButton.whenPressed(new ShooterMotorHigh(m_turret));
    HoodSolenoidButton.whenPressed(new hoodSolenoid(m_turret));
    kickbutton.whenPressed(new Kick(m_harvester));
   // LowShootButton.whenPressed(new ShooterMotorLow(m_turret));
    ReverseAgitate.whileHeld(new ReverseAgitate(m_agitator));
    ReverseAgitate.whenReleased(new AgitatorStop(m_agitator));
    TurretSolenoidButton.whenPressed(new TurretSolenoid(m_turret));
 new JoystickButton(stick1, Button.kBumperLeft.value)
    .whenPressed(new InstantCommand(m_shooter::enable, m_shooter));
new JoystickButton(stick1, Button.kBumperRight.value)
    .whenPressed(new InstantCommand(m_shooter::disable, m_shooter));
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

  public boolean ToggleLoop(){
        //Set current button value;
        isShooterPressed = PIDShootButton.get();

        //if value has changed
        if(lastShooterVal != isShooterPressed){
          lastShooterVal = isShooterPressed;
          //if we are currently pressing it
          if(isShooterPressed == true){
            Toggler = !Toggler;
          //tell shooter to initialize
            if(Toggler == true){
              m_shooter.InitLoop();
            }
            else{
              m_shooter.EndConst();
            }
          }
        }
      return Toggler;
  }

}
