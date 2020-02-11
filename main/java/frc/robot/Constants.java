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
    public static final int AutoTurnButton = 3; // X Button
    public static final int ColorwheelButton = 10; // Right Stick
    public static final int Harvest = 7; // Back Button
    public static final int HighShootButton = 4; // Y Button
    public static final int KickButton = 1;
    public static final int LowerSensButton = 5; // Left Bumper
    public static final int LowShootButton = 8; // Start Button
    public static final int RaiseSensButton = 6; // Right Bumper
   // public static final int ShifterButton = 1; // A Button
    
    //Controller Two Inputs
    public static final int XboxPort1 = 1;
    public static final int Colored = 1; //A Button
    public static final int HoodSolenoidButton = 7; // Back Button

    //Chassis Motors
    public static final int PigeonSRX = 14;
    public static final int LeftFront = 1;
    public static final int LeftBack = 2;
    public static final int RightFront = 3;
    public static final int RightBack = 4;

    //Other Motors
    public static final int Agitate = 0;
    public static final int BottomKickMotor = 18;
    public static final int ClimbMotor = 0;
    public static final int ColorwheelMotor = 14;
    public static final int HarvesterMotor = 15;
    public static final int ShooterMotor = 17;
    public static final int ShooterMotor2 = 14;
    public static final int TopKickMotor = 10;
    public static final int TurretMotor = 16;
    
    //Pneumatics
    public static final int ClimbSolenoid = 0;
    public static final int GearShift = 7;
    public static final int HoodSolonoid = 0;
	public static final int Harvestsolenoid = 0;

    //Encoder
    public static final int Encoder = 0;

    //Autonomous Distance
    public static final double ksVolts = 0.309;
    public static final double kvVoltSecondsPerMeter = 0.143;
    public static final double kaVoltSecondsSquaredPerMeter = 0.00603;

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
