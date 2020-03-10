/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;


public class ShooterSubsystem extends PIDSubsystem {
  private final WPI_TalonFX m_shooterMotor = new WPI_TalonFX(Constants.ShooterMotor1);
  private final WPI_TalonFX m_shooterMotor2 = new WPI_TalonFX(Constants.ShooterMotor2);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(Constants.kSVolts, Constants.kVVoltSecondsPerRotation);

  /**
   * The shooter subsystem for the robot.
   */
  public ShooterSubsystem() {
    super(new PIDController(Constants.kP, Constants.kI, Constants.kD));
    getController().setTolerance(Constants.kShooterToleranceRPS);
    setSetpoint(Constants.kShooterTargetRPS);
    m_shooterMotor2.follow(m_shooterMotor);
    m_shooterMotor2.setInverted(true);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    return m_shooterMotor.getSelectedSensorVelocity();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
