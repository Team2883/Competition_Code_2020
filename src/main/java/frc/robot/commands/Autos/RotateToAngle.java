/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import java.lang.Math;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RotateToAngle extends CommandBase
{
    double angleToRotate = 0;
    double dRMax = 0;
    double dRMin = 0;
    double yaw;
    Timer timer;
    AHRS ahrs;
    private double driveRotation;
    private boolean HasReset;
    private boolean HasFinished;
    private final DriveTrain m_driveTrain;

    public RotateToAngle(double angle, double rotationMax, double rotationMin, DriveTrain subsystem)
    {
        m_driveTrain = subsystem;
        angleToRotate = angle;
        dRMax = rotationMax;
        dRMin = rotationMin;
        driveRotation = rotationMax;
        timer = new Timer();
       // ahrs = new AHRS(SerialPort.Port.kOnboard);
        addRequirements(m_driveTrain);
	}

    @Override
    public void execute()
    {
       // yaw = m_driveTrain.getHeading();
        SmartDashboard.putNumber("AHRS Angle", Robot.Yaw);
        if(!HasReset){
           m_driveTrain.zeroHeading();
            HasReset = true;
            HasFinished = false;
            timer.reset();
            timer.start();
        }
        else{//speed calculation
        driveRotation = (angleToRotate - Robot.Yaw) / angleToRotate * dRMax;
        if(driveRotation > dRMax)
        {
            driveRotation = dRMax;
        }
        else if (driveRotation < dRMin)
        {
            driveRotation = dRMin;
        }
        //Actual rotation
        if(angleToRotate < 0)
        {
            if(angleToRotate < Robot.Yaw){
                m_driveTrain.Drive(0, driveRotation);
                timer.reset();
            }
            else if(Math.abs(angleToRotate - Robot.Yaw) > 3){
                m_driveTrain.Drive(0, -driveRotation);
                timer.reset();
            }
            else if(timer.get() >  0.5)
                HasFinished = true;
        }
        else if(angleToRotate > 0)
        {
            if(angleToRotate > Robot.Yaw){
                m_driveTrain.Drive(0, -driveRotation);
                timer.reset();
            }
            else if(Math.abs(angleToRotate - Robot.Yaw) > 3){
                m_driveTrain.Drive(0, driveRotation);
                timer.reset();
            }
            else if(timer.get() >  0.5)
                HasFinished = true;
        }
        }
        System.out.print(timer.get());
    }

	@Override
    public boolean isFinished() 
    {
        if(HasFinished == true)
            System.out.println("Finished Rotation.");
        return HasFinished;
    }

    @Override
    public void end(boolean inturrupted)
    {
        m_driveTrain.m_dDrive.tankDrive(0, 0);
        HasFinished = false;
        HasReset = false; 
        timer.reset();
        timer.stop();
    }
}
