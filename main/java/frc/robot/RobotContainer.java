/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Colors.Colored;
import frc.robot.commands.Drives.StickDrive;
import frc.robot.commands.HarvestandFeed.Harvest;
import frc.robot.commands.HarvestandFeed.Kick;
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
  private final Harvester m_harvester = new Harvester();

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
    final JoystickButton AutoTurnButton = new JoystickButton(stick, Constants.AutoTurnButton);
    final JoystickButton Harvest = new JoystickButton(stick, Constants.Harvest);
    final JoystickButton HighShootButton = new JoystickButton(stick, Constants.HighShootButton);
    final JoystickButton kickbutton = new JoystickButton(stick, Constants.KickButton);
    final JoystickButton LowerSensButton = new JoystickButton(stick, Constants.LowerSensButton);
    final JoystickButton LowShootButton = new JoystickButton(stick, Constants.LowShootButton);
    final JoystickButton RaiseSensButton = new JoystickButton(stick, Constants.RaiseSensButton);

    final JoystickButton Colored = new JoystickButton(stick1, Constants.Colored);
    final JoystickButton HoodSolenoidButton = new JoystickButton(stick1, Constants.HoodSolenoidButton);


    AutoTurnButton.whileHeld(new TurretVision(m_turret));
    Harvest.whenPressed(new Harvest (m_harvester));
    HighShootButton.whenPressed(new ShooterMotorHigh(m_turret));
    kickbutton.whenPressed(new Kick(m_harvester));
    LowerSensButton.whenPressed(new sensDown(m_driveTrain));
    LowShootButton.whenPressed(new ShooterMotorLow(m_turret));
    RaiseSensButton.whenPressed(new sensUp(m_driveTrain));
    //ShifterButton.whenPressed(new Shift(m_driveTrain));

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
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(Constants.ksVolts,
      Constants.kvVoltSecondsPerMeter,
      Constants.kaVoltSecondsSquaredPerMeter),
    Constants.kDriveKinematics,
    10);

    // Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(-180)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
        new Translation2d(1, 1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config);

    String trajectoryJSON = "paths/Trenchrun.wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      SmartDashboard.putBoolean("haspathbeenfound", true);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      SmartDashboard.putBoolean("haspathbeenfound", false);
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
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
      m_driveTrain);

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
