// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandConstants;

public class HandSubsystem extends SubsystemBase {
  /** Creates a new HandSubsystem. */
  private SparkMax m_motorHand;
  private SparkMaxConfig m_motorConfigHand;
  private RelativeEncoder m_encoderHand;
  private SparkClosedLoopController m_closedLoopHand;

  public HandSubsystem() {
    m_motorHand = new SparkMax(HandConstants.motorHand, MotorType.kBrushless);
      m_motorConfigHand = new SparkMaxConfig();
      m_motorConfigHand.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HandConstants.kCurrentLimit)
        .secondaryCurrentLimit(HandConstants.kSecondaryCurrentLimit);
      m_motorConfigHand.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
      m_motorConfigHand.closedLoop.maxMotion
        .maxAcceleration(1000)
        .maxVelocity(2000)
        .allowedClosedLoopError(1);
      m_closedLoopHand = m_motorHand.getClosedLoopController();
      m_motorHand.configure(m_motorConfigHand, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_encoderHand = m_motorHand.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber( "HandPostion", m_encoderHand.getPosition());
    SmartDashboard.putNumber("Hand Current",m_motorHand.getOutputCurrent());
  }

  public boolean isHandStalled() {
   return m_motorHand.getOutputCurrent()>= HandConstants.kHandStallCurrent;
  } 

  public void stopHandMotor() {
    m_motorHand.set(0);
  }

  public Command cmdStopHand() {
    return Commands.runOnce(() -> stopHandMotor() , this);
  }
}