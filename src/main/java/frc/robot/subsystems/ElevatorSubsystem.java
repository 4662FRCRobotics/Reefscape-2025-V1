// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Configs.ElevatorMotor;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  public enum ElevatorLevel {
    kL1 (17.88),
    kL2 (31.72),
    kL3 (47.59),
    kL4 (71.87);

    public double m_positionInches;
    public double m_positionEncoder;

    private ElevatorLevel(double positionInches) {
      this.m_positionInches = positionInches;
      this.m_positionEncoder = ((positionInches - ElevatorConstants.kHandStartUpInches) / ElevatorConstants.kWinchCircumferenceInches) * ElevatorConstants.kGearRatio;
    }

    public double getPositionInches() {
      return m_positionInches;
    }
  }

  private SparkMax m_motorElevatorLeft;
  private SparkMax m_motorElevatorRight;
  private SparkMaxConfig m_motorConfigRight;
  private SparkClosedLoopController m_closedLoopElevatorLeft;
  private SparkMaxConfig m_motorConfigLeft;
  private double m_elevatorTargetPostion;
  private RelativeEncoder m_encoderElevatorLeft;
  private double m_elevatorP;
  private double m_elevatorI;
  private double m_elevatorD;
  private SoftLimitConfig m_motorSoftLimitLeft;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorP = 0.4;
    m_elevatorI = 0;
    m_elevatorD = 0;
    m_motorElevatorLeft = new SparkMax(ElevatorConstants.motorElevatorLeft, MotorType.kBrushless);
      m_motorConfigLeft = new SparkMaxConfig();
      m_motorConfigLeft.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
        .secondaryCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit);
      m_motorConfigLeft.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(m_elevatorP)
        .i(m_elevatorI)
        .d(m_elevatorD)
        .outputRange(-1, 1);
      m_motorConfigLeft.closedLoop.maxMotion
        .maxAcceleration(2500)
        .maxVelocity(5000)
        .allowedClosedLoopError(1);
      m_motorSoftLimitLeft = new SoftLimitConfig();
      m_motorSoftLimitLeft.forwardSoftLimit(ElevatorConstants.kFwdSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ElevatorConstants.kRevSoftLimit)
        .reverseSoftLimitEnabled(true);
        
    m_closedLoopElevatorLeft = m_motorElevatorLeft.getClosedLoopController();
    m_motorElevatorLeft.configure(m_motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motorElevatorRight = new SparkMax(ElevatorConstants.motorElevatorRight, MotorType.kBrushless);
    m_motorConfigRight = new SparkMaxConfig();
    m_motorConfigRight
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
      .secondaryCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit)
      .follow(ElevatorConstants.motorElevatorLeft,true);
    m_motorElevatorRight.configure(m_motorConfigRight,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoderElevatorLeft = m_motorElevatorLeft.getEncoder();

    m_elevatorTargetPostion = ElevatorConstants.kHandStartUpInches;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Current",m_motorElevatorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right Current",m_motorElevatorRight.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Position",m_elevatorTargetPostion);
    SmartDashboard.putNumber("Actual Pos", getPositionInches());
    SmartDashboard.putNumber("Elevator P" , m_elevatorP);
    SmartDashboard.putNumber( "Elevator I" , m_elevatorI);
    SmartDashboard.putNumber( "Elevator D" , m_elevatorD);
  }

  private void updateElevatorConfig() {
    m_elevatorP = SmartDashboard.getNumber("Elevator P" , m_elevatorP);
    m_elevatorI = SmartDashboard.getNumber("Elevator I" , m_elevatorI);
    m_elevatorD = SmartDashboard.getNumber("Elevator D" , m_elevatorD);
    m_motorConfigLeft.closedLoop
        //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(m_elevatorP)
        .i(m_elevatorI)
        .d(m_elevatorD)
        .outputRange(-1, 1);
  }

  public Command cmdUpdateElevatorConfig() {
    return Commands.runOnce(() -> updateElevatorConfig() , this).ignoringDisable(true);
  }

  private void setElevatorPosition(double targetPosition) {
    m_elevatorTargetPostion = targetPosition;
    double targetEncoderPosition = ((m_elevatorTargetPostion - ElevatorConstants.kHandStartUpInches) / ElevatorConstants.kWinchCircumferenceInches) * ElevatorConstants.kGearRatio;
    m_closedLoopElevatorLeft.setReference(targetEncoderPosition, ControlType.kMAXMotionPositionControl);
  }

  private void adjustElevatorPosition(boolean isAdjustUp) {
    if (isAdjustUp) {
      m_elevatorTargetPostion = m_elevatorTargetPostion + ElevatorConstants.kPostionAdjust;
    } else {
      m_elevatorTargetPostion = m_elevatorTargetPostion - ElevatorConstants.kPostionAdjust;
    }
    setElevatorPosition(m_elevatorTargetPostion);
  //  m_closedLoopElevatorLeft.setReference(m_elevatorTargetPostion, ControlType.kMAXMotionPositionControl);
  }

  public Command cmdAdjustElevatorPosition(boolean isAdjustUp) {
    return Commands.runOnce(() -> adjustElevatorPosition(isAdjustUp) , this);
  }

  public Command cmdSetElevatorPosition(ElevatorLevel elevatorLevel) {
    return Commands.runOnce(() -> setElevatorPosition(elevatorLevel.getPositionInches()) , this);
  }

  public boolean isElevatorStalled(){
    return m_motorElevatorLeft.getOutputCurrent() >= ElevatorConstants.kStallCurrent;
  }

  private void stopElevatorMotor(){
    m_motorElevatorLeft.set(0);
  }

  public Command cmdStopElevator() {
    return Commands.runOnce(() -> stopElevatorMotor() , this);
  }

  public boolean isElevatorUp() {
    return getPositionInches() > ElevatorLevel.kL2.getPositionInches();
  }

  public boolean isElevatorInPickup() {
    return getPositionInches() <= ElevatorConstants.kCoralPickup;
  }

  public boolean isElevatorAtCrossbar() {
    return getPositionInches() >= ElevatorConstants.kCrossbar;
  }

  private double getPositionInches() {
    return (( m_encoderElevatorLeft.getPosition() / ElevatorConstants.kGearRatio) * ElevatorConstants.kWinchCircumferenceInches) + ElevatorConstants.kHandStartUpInches;
  }

  private void elevatorZero(){
    stopElevatorMotor();
    m_encoderElevatorLeft.setPosition(0);
  }
  public Command cmdElevatorZero(){
    return Commands.runOnce(() -> elevatorZero(), this);

  }
}
