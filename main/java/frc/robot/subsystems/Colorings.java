/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

 package frc.robot.subsystems;

 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.wpilibj.I2C;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj.util.Color;
 import com.revrobotics.ColorSensorV3;
 import com.revrobotics.ColorMatchResult;
 import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
 import com.revrobotics.ColorMatch;
 import frc.robot.Constants;


 public class Colorings extends SubsystemBase 
 {
   final public WPI_TalonSRX m_ColorWheel = new WPI_TalonSRX(Constants.ColorwheelMotor);
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
   private final ColorMatch m_colorMatcher = new ColorMatch();

   private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
   private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
   private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
   private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

   Color detectedColor = m_colorSensor.getColor();
   ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
   boolean done = false;
   boolean in = false;

   public void setColorTargets() 
   {
     m_colorMatcher.addColorMatch(kBlueTarget);
     m_colorMatcher.addColorMatch(kGreenTarget);
     m_colorMatcher.addColorMatch(kRedTarget);
     m_colorMatcher.addColorMatch(kYellowTarget); 
   }

   @Override
   public void periodic() 
   {
     Color detectedColor = m_colorSensor.getColor();
     String colorString;
     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

     if (match.color == kBlueTarget) {
       colorString = "Blue";
     } 
     else if (match.color == kRedTarget) {
       colorString = "Red";
     }
     else if (match.color == kGreenTarget) {
       colorString = "Green";
     } 
     else if (match.color == kYellowTarget) {
       colorString = "Yellow";
     } 
     else  {
       colorString = "Unknown";
     }

    SmartDashboard.putNumber("Blue", detectedColor.blue);
     SmartDashboard.putNumber("Green", detectedColor.green);
     SmartDashboard.putNumber("Red", detectedColor.red);
     SmartDashboard.putNumber("Confidence", match.confidence);
     SmartDashboard.putString("Detected Color", colorString);
   }

   public void SpinWheel(double speed)
   {
     m_ColorWheel.set(speed);
   }

   public void Blue()
   {
     setColorTargets();
     Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

     if (match.color == kRedTarget)
     {
     SmartDashboard.putBoolean("Target Found", true);
     }
     else
     {
       SmartDashboard.putBoolean("Target Found", false);
     }
   }

   public void Green()
   {
     setColorTargets();
     Color detectedColor = m_colorSensor.getColor();
     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
     if (match.color == kYellowTarget)
     {
       SmartDashboard.putBoolean("Target Found", true);
       SpinWheel(0);
     }
     else
     {
       SmartDashboard.putBoolean("Target Found", false);
       SpinWheel(.3);
     }
   }

   public void Red()
   {
     setColorTargets();
     Color detectedColor = m_colorSensor.getColor();
     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
     if (match.color == kBlueTarget)
     {
       SmartDashboard.putBoolean("Target Found", true);
       SpinWheel(0);
     }
     else 
     {
       SmartDashboard.putBoolean("Target Found", false);
       SpinWheel(.3);
     }
   }

   public void Yellow()
   {
     setColorTargets();
     Color detectedColor = m_colorSensor.getColor();
     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
     if (match.color == kGreenTarget)
     {
       SmartDashboard.putBoolean("Target Found", true);
       SpinWheel(0);
     }
     else 
     {
       SmartDashboard.putBoolean("Target Found", false);
       SpinWheel(.3);
     }
   }
}
