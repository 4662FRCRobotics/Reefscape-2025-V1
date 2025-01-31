// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Configs.ElevatorMotor;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax m_motorElevatorLeft;
  private SparkMax m_motorElevatorRight;
  private SparkMaxConfig m_motorConfigRight;
  private SparkClosedLoopController m_closedLoopElevatorLeft;
  private SparkMaxConfig m_motorConfigLeft;
  private double m_elevatorTargetPostion;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motorElevatorLeft = new SparkMax(ElevatorConstants.motorElevatorLeft, MotorType.kBrushless);
      m_motorConfigLeft = new SparkMaxConfig();
      m_motorConfigLeft.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
        .secondaryCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit);
      m_motorConfigLeft.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
      m_motorConfigLeft.closedLoop.maxMotion
        .maxAcceleration(1000)
        .maxVelocity(2000)
        .allowedClosedLoopError(1);
        
    m_closedLoopElevatorLeft = m_motorElevatorLeft.getClosedLoopController();
    m_motorElevatorLeft.configure(m_motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_motorElevatorRight = new SparkMax(ElevatorConstants.motorElevatorRight, MotorType.kBrushless);
    m_motorConfigRight = new SparkMaxConfig();
    m_motorConfigRight
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
      .secondaryCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit)
      .follow(ElevatorConstants.motorElevatorLeft,true);
    m_motorElevatorRight.configure(m_motorConfigRight,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_elevatorTargetPostion = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Current",m_motorElevatorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right Current",m_motorElevatorRight.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Position",m_elevatorTargetPostion);
  }

  public void setElevatorPosition(double targetPosition) {
    m_elevatorTargetPostion = targetPosition;
    m_closedLoopElevatorLeft.setReference(m_elevatorTargetPostion, ControlType.kMAXMotionPositionControl);
  }

  private void adjustElevatorPosition(boolean isAdjustUp) {
    if (isAdjustUp) {
      m_elevatorTargetPostion = m_elevatorTargetPostion + ElevatorConstants.kPostionAdjust;
    } else {
      m_elevatorTargetPostion = m_elevatorTargetPostion - ElevatorConstants.kPostionAdjust;
    }
    m_closedLoopElevatorLeft.setReference(m_elevatorTargetPostion, ControlType.kMAXMotionPositionControl);
  }

  public Command cmdAdjustElevatorPosition(boolean isAdjustUp) {
    return Commands.runOnce(() -> adjustElevatorPosition(isAdjustUp) , this);
  }

  public Command cmdSetElevatorPosition(double targetPosition) {
    return Commands.runOnce(() -> setElevatorPosition(targetPosition) , this);
  }

  public void runMotor(double speed){
    m_motorElevatorLeft.set(speed);
  }

  public boolean isElevatorStalled(){
    return m_motorElevatorLeft.getOutputCurrent() >=20;
  }

  public void stopElevatorMotor(){
    m_motorElevatorLeft.set(0);
  }

  public Command cmdStopElevator() {
    return Commands.runOnce(() -> stopElevatorMotor() , this);
  }

}
