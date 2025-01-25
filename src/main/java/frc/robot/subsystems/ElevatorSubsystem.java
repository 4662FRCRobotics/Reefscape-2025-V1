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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ElevatorMotor;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax m_motorElevatorLeft;
  private SparkClosedLoopController m_closedLoopElevatorLeft;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motorElevatorLeft= new SparkMax(ElevatorConstants.motorElevatorLeft, MotorType.kBrushless);
    m_closedLoopElevatorLeft = m_motorElevatorLeft.getClosedLoopController();
    m_motorElevatorLeft.configure(ElevatorMotor.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setElevatorPosition(double targetPosition) {
    System.out.println(targetPosition);
    m_closedLoopElevatorLeft.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }

  public Command cmdSetElevatorPosition(double targetPosition) {
    return Commands.run(() -> setElevatorPosition(targetPosition) , this);
  }

}
