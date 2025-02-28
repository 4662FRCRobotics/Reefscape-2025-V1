// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraServoConstants;

public class CameraServoSubsystem extends SubsystemBase {
  /** Creates a new CameraServoSubsystem. */
  private Servo m_CameraServo;
  private double m_CameraAngle;

  public CameraServoSubsystem() {
    m_CameraServo = new Servo(CameraServoConstants.servoPort);
  }

  public void setCameraAngle(double controlAxis) {
    m_CameraAngle = (controlAxis + 1) * 90;
    //System.out.println(m_CameraAngle);
    m_CameraServo.setAngle(m_CameraAngle);
  }

  public Command cmdCameraAngle(double controlAxis) {
    return Commands.run(() -> setCameraAngle(controlAxis) , this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   SmartDashboard.putNumber("Camera Position" , m_CameraAngle);
  }
}
