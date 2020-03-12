/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import imports.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
//Shooter Imports
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource.*;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;




public class ShooterSubsystem extends PIDSubsystem {
  double actual_RPM;
  private final WPI_TalonFX m_shooterMotor1 = new WPI_TalonFX(Constants.ShooterMotor1);
  private final WPI_TalonFX m_shooterMotor2 = new WPI_TalonFX(Constants.ShooterMotor2);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(Constants.kSVolts, Constants.kVVoltSecondsPerRotation);
  /**
   * The shooter subsystem for the robot.
   */
  public ShooterSubsystem() {
    super(new PIDController(Constants.kP, Constants.kI, Constants.kD));
    //getController().setTolerance(Constants.kShooterToleranceRPS);
    //setSetpoint(Constants.kShooterTargetRPS);
    m_shooterMotor1.configFactoryDefault();
    m_shooterMotor2.configFactoryDefault();
    m_shooterMotor1.setNeutralMode(NeutralMode.Coast);
		m_shooterMotor2.setNeutralMode(NeutralMode.Coast);
    
    m_shooterMotor1.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    m_shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                      Constants.REMOTE_0, 
											Constants.kTimeoutMs);
    m_shooterMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                      Constants.REMOTE_0, 
                      Constants.kTimeoutMs);									

		/* Config the peak and nominal outputs */
		m_shooterMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
		m_shooterMotor1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		m_shooterMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_shooterMotor1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		m_shooterMotor2.configNominalOutputForward(0, Constants.kTimeoutMs);
		m_shooterMotor2.configNominalOutputReverse(0, Constants.kTimeoutMs);
		m_shooterMotor2.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_shooterMotor2.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/* Config the Velocity closed loop gains in slot0 */
		m_shooterMotor1.config_kF(Constants.REMOTE_0, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		m_shooterMotor1.config_kP(Constants.REMOTE_0, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		m_shooterMotor1.config_kI(Constants.REMOTE_0, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
    m_shooterMotor1.config_kD(Constants.REMOTE_0, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
    m_shooterMotor2.config_kF(Constants.REMOTE_0, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		m_shooterMotor2.config_kP(Constants.REMOTE_0, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		m_shooterMotor2.config_kI(Constants.REMOTE_0, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		m_shooterMotor2.config_kD(Constants.REMOTE_0, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);

    m_shooterMotor2.setInverted(true);
		/** USE THIS TO INCREASE THE SENSOR READ RATE!!
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
    //final int closedLoopTimeMs = 10;
    //m_shooterMotor1.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
   // m_shooterMotor1.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

    SmartDashboard.putString("ShooterActive", "Created");
  }

  @Override
  public void useOutput(final double output, final double setpoint) {
    m_shooterMotor1.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_shooterMotor1.getSelectedSensorVelocity();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }


  public void InitLoop(){
    System.out.println("This is Velocity Closed Loop with an Arbitrary Feed Forward.");
    //System.out.println("Travel [-500, 500] RPM while having the ability to add a FeedForward with joyX ");
    zeroSensors();

    /* Determine which slot affects which PID */
    //m_shooterMotor1.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
  }


  public void zeroSensors(){

    m_shooterMotor1.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		m_shooterMotor2.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
  }

  public double GetRPM(){
    return actual_RPM;
  }

  public void HoldConstant(){
  /* Calculate targets from gamepad inputs */
  //double target_RPM = Constants.shooterPercentage * Constants.shooterTargetRPM;	// +- 2000 RPM
  double target_unitsPer100ms = Constants.TargetSensorValue;	//RPM -> Native units
  //double feedFwdTerm = Constants.shooterFeed * Constants.shooterFeedForwardMod;	// Percentage added to the close loop output

  /* Configured for Velocity Closed Loop on Quad Encoders' Sum and Arbitrary FeedForward on joyX */
  m_shooterMotor1.set(TalonFXControlMode.Velocity, target_unitsPer100ms);
  m_shooterMotor2.follow(m_shooterMotor1);

  //tracking RPMS
  actual_RPM = (m_shooterMotor1.getSelectedSensorVelocity() / (double)Constants.kEncoderCPR * 600f);
  double shooterVal = m_shooterMotor1.getSelectedSensorVelocity();
  // double position = m_shooterMotor1.getSelectedSensorPosition(pidIdx)
  SmartDashboard.putNumber("Current RPMS: ",actual_RPM);
  SmartDashboard.putNumber("Sensor Reading", shooterVal);
  SmartDashboard.putNumber("Target Sensor Value", target_unitsPer100ms);

}


public void EndConst(){
  m_shooterMotor1.setVoltage(0);
  m_shooterMotor2.setVoltage(0);
}

}
