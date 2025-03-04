// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.libraries.CommandXBoxOpControl;
import frc.robot.libraries.ConsoleAuto;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.CameraServoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final HandSubsystem m_HandSubsystem = new HandSubsystem();
  private final CameraServoSubsystem m_CameraServoSubsystem = new CameraServoSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXBoxOpControl m_operatorController = new CommandXBoxOpControl(OIConstants.kOperatorControllerPort);

  private final ConsoleAuto m_ConsoleAuto = 
    new ConsoleAuto(OIConstants.kAUTONOMOUS_CONSOLE_PORT);
  
  private final CommandGenericHID m_CommandGenericHID = 
    new CommandGenericHID(OIConstants.kTeleopConsolePort);

  private final AutonomousSubsystem m_AutonomousSubsystem = new AutonomousSubsystem(m_ConsoleAuto, this);

  static boolean m_runAutoConsole;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

     //Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false,
                m_elevator.isElevatorUp()),
            m_robotDrive));
    
    m_CameraServoSubsystem.setDefaultCommand(
      //m_CameraServoSubsystem.cmdCameraAngle(m_CommandGenericHID.getRawAxis(0))
     new RunCommand(
      () -> m_CameraServoSubsystem.setCameraAngle(m_CommandGenericHID.getRawAxis(0)) , m_CameraServoSubsystem)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(m_driverController, Button.kR1.value)
    m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
     runAutoConsoleFalse();
    //new Trigger(DriverStation::isDisabled)
    //new Trigger(RobotModeTriggers.disabled())
    new Trigger(trgAutoSelect())
      .whileTrue(m_AutonomousSubsystem.cmdAutoSelect());
    runAutoConsoleTrue();

    new Trigger(RobotModeTriggers.disabled())
      .onTrue(Commands.runOnce(this::runAutoConsoleTrue)
        .ignoringDisable(true))
      ;

    new Trigger(RobotModeTriggers.disabled())
    .onFalse(Commands.runOnce(this::runAutoConsoleFalse))
    ;
  
    m_operatorController.a() 
        .onTrue(m_elevator.cmdSetElevatorPosition(ElevatorConstants.kTrough));
    m_operatorController.x()
        .onTrue(m_elevator.cmdSetElevatorPosition(ElevatorConstants.kLevel2));
    m_operatorController.b()
        .onTrue(m_elevator.cmdSetElevatorPosition(ElevatorConstants.kLevel3));
    m_operatorController.y()
        .onTrue(m_elevator.cmdSetElevatorPosition(ElevatorConstants.kLevel4));
    m_operatorController.leftBumper()
        .whileTrue(Commands.run(()->m_elevator.runMotor(0)));
    m_operatorController.rightBumper()
        .onTrue(m_elevator.cmdElevatorZero());
    m_operatorController.povUp()
        .onTrue(m_elevator.cmdAdjustElevatorPosition(true));
    m_operatorController.povDown()
        .onTrue(m_elevator.cmdAdjustElevatorPosition(false));
    new Trigger(() -> m_elevator.isElevatorStalled())
        .onTrue(m_elevator.cmdStopElevator());
    m_operatorController.leftTrigger()
        .onTrue(m_HandSubsystem.cmdSetHandPosition().unless(() -> m_elevator.isElevatorInPickup()));
    m_operatorController.povLeft()
        .onTrue(m_HandSubsystem.cmdAdjustHandPosition(true));
    m_operatorController.povRight()
        .onTrue(m_HandSubsystem.cmdAdjustHandPosition(false));
    m_CommandGenericHID.button(1)
        .onTrue(m_elevator.cmdUpdateElevatorConfig());

  }

  private static Trigger trgAutoSelect() {
    //System.out.println("bool auto console" + m_runAutoConsole);
    return new Trigger(() -> m_runAutoConsole);
  }

  private void runAutoConsoleTrue() {
    m_runAutoConsole = true;
    System.out.println("true " + m_runAutoConsole);
  }

  private void runAutoConsoleFalse() {
    m_runAutoConsole = false;
    System.out.println("false " + m_runAutoConsole);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return m_robotDrive.getPathStep("drive out - Auto");
    return m_AutonomousSubsystem.cmdAutoControl();
  }

  public Command getDrivePlanCmd(String planName) {
    return m_robotDrive.getPathStep(planName);
  }
}
