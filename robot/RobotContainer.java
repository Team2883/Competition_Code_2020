/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Colors.Colored;
import frc.robot.commands.Drives.Shift;
import frc.robot.commands.Drives.StickDrive;
import frc.robot.commands.HarvestandFeed.Agitator;
import frc.robot.commands.HarvestandFeed.Harvest;
import frc.robot.commands.HarvestandFeed.Kick;
import frc.robot.commands.HarvestandFeed.KickUp;
import frc.robot.commands.Sensors.sensDown;
import frc.robot.commands.Sensors.sensUp;
import frc.robot.commands.TurretandShooter.ShooterMotorHigh;
import frc.robot.commands.TurretandShooter.ShooterMotorLow;
import frc.robot.commands.TurretandShooter.TurretVision;
import frc.robot.commands.TurretandShooter.hoodSolenoid;
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

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Turret m_turret = new Turret();
  private final Colorings m_Colorings = new Colorings();
  private final Harvester m_Harvester = new Harvester();
  private Trajectory trajectory;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
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
  }
  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    //final JoystickButton ShifterButton = new JoystickButton(stick, Constants.ShifterButton);
    final JoystickButton LowerSensButton = new JoystickButton(stick, Constants.LowerSensButton);
    final JoystickButton RaiseSensButton = new JoystickButton(stick, Constants.RaiseSensButton);
    final JoystickButton AutoTurnButton = new JoystickButton(stick, Constants.AutoTurnButton);
    final JoystickButton HighShootButton = new JoystickButton(stick, Constants.HighShootButton);
    //final JoystickButton LowShootButton = new JoystickButton(stick, Constants.LowShootButton);
    final JoystickButton Harvest = new JoystickButton(stick, Constants.Harvest);
    final JoystickButton kickupButton = new JoystickButton(stick, Constants.kickupButton);
    final JoystickButton kickbutton = new JoystickButton(stick, Constants.kickbutton);
    final JoystickButton AgitatorButton = new JoystickButton(stick, Constants.AgitatorButton);

    final JoystickButton Colored = new JoystickButton(stick1, Constants.Colored);
    final JoystickButton HoodSolenoidButton = new JoystickButton(stick1, Constants.HoodSolenoidButton);


    LowerSensButton.whenPressed(new sensDown(m_driveTrain));
    RaiseSensButton.whenPressed(new sensUp(m_driveTrain));
    //ShifterButton.whenPressed(new Shift(m_driveTrain));
    AutoTurnButton.whileHeld(new TurretVision(m_turret));
    HighShootButton.whenPressed(new ShooterMotorHigh(m_turret));
    kickupButton.whenPressed(new KickUp(m_turret));
    //LowShootButton.whenPressed(new ShooterMotorLow(m_turret));
    Harvest.whenPressed(new Harvest (m_Harvester));
    kickbutton.whenPressed(new Kick(m_turret));
    AgitatorButton.whenPressed(new Agitator(m_turret));
    Colored.whileHeld(new Colored(m_Colorings));
    HoodSolenoidButton.whenPressed(new hoodSolenoid(m_turret));
  }
  
  /**
   * Use this to pass the autoDrivenomous command to the main {@link Robot} class.
   *
   *  the command to run in autoDrivenomous
   */
  public Command getAutonomousCommand() 
  {
    String trajectoryJSON = "paths/Trenchrun.wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      m_driveTrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts,
      Constants.kvVoltSecondsPerMeter,
      Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      m_driveTrain::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_driveTrain::tankDriveVolts,
      m_driveTrain
      );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0));
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
