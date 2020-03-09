/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    //Controller One Inputs
    public static final int XboxPort = 0;
    public static final int ClimbSolenoidButton = 7; // Back Button
    public static final int ClimbReverseButton = 2; // B Button
    public static final int Colored = 3; // X Button
    public static final int ColorsSolenoid = 5; // Left Bumper
    public static final int Harvest = 1; // A Button
    public static final int HarvestSolenoid = 6; // Right Bumper
    public static final int ShifterButton = 10; // Right Stick

    //Controller Two Inputs
    public static final int XboxPort1 = 1;
    public static final int Agitator = 1; // A Button
    public static final int AutoTurnButton = 3; // X Button
    public static final int LowShootButton = 10; // Start Button
    public static final int HighShootButton = 4; // Y Button
    public static final int Hoodsolenoid = 8; // Start Button
    public static final int KickButton = 2; // B Button
    public static final int ReverseAgitate = 7; // Right Bumper
    public static final int TurretSolenoidButton = 7; // Back Button

    //Chassis Motors
    public static final int LeftBack = 0;
    public static final int LeftFront = 1;
    public static final int RightBack = 2;
    public static final int RightFront = 3;

    //Other Motors
    public static final int Agitate = 12;
    public static final int BottomKickMotor = 18;
    public static final int ClimbMotor = 4;
    public static final int ColorwheelMotor = 11;
    public static final int HarvesterMotor = 15;
    public static final int ShooterMotor = 5;
    public static final int ShooterMotor2 = 14;
    public static final int TopKickMotor = 10;
    public static final int TurretMotor = 16;
    
    //Pneumatics
    public static final int ClimbSolenoid = 0;
    public static final int ClimbSolenoid1 = 1;
    public static final int GearShift = 5;
    public static final int HoodSolenoid = 4;
    public static final int TurretSolenoid = 7;
	public static final int Harvestsolenoid1 = 2;
    public static final int Harvestsolenoid2 = 3;
    public static final int ColorSolenoid = 6;

    //Sensors
    public static final int LeftSwitch = 0;
    public static final int RightSwitch = 1;

    //Autonomous Distance
    public static final double ksVolts = 0.00929;
    public static final double kvVoltSecondsPerMeter = 5.86;
    public static final double kaVoltSecondsSquaredPerMeter = 0.357;

    // Shooter Constants
    public static final int[] kEncoderPorts = new int[]{4, 5};
    public static final boolean kEncoderReversed = false;
    public static final int kEncoderCPR = 2048;
    public static final double kShooterEncoderDistancePerPulse =
        // Distance units will be rotations
        1.0 / (double) kEncoderCPR;

    public static final int kShooterMotorPort = 4;
    public static final int kFeederMotorPort = 5;

    public static final double kShooterFreeRPS = 5300;
    public static final double kShooterTargetRPS = 1;
    public static final double kShooterToleranceRPS = 50;

    // These are not real PID gains, and will have to be tuned for your specific robot.
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    // On a real robot the feedforward constants should be empirically determined; these are
    // reasonable guesses.
    public static final double kSVolts = 0.05;
    public static final double kVVoltSecondsPerRotation =
        // Should have value 12V at free speed...
        12.0 / kShooterFreeRPS;

    public static final double kFeederSpeed = 0.5;
    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.5;
    public static final double kTrackwidthMeters = 0.527;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
	public static final boolean kGyroReversed = true;
	public static final String kEncoderDistancePerPulse = "0.000019347";
}
