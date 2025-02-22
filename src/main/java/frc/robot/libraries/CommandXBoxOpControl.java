// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class CommandXBoxOpControl extends CommandXboxController{

    public CommandXBoxOpControl(int port) {
        super(port);
    }

    //copied from Command Xbox Controller docs
    public Trigger rightYUpTrigger(double threshold, EventLoop loop) {
        return axisLessThan(XboxController.Axis.kRightY.value, threshold, loop);
    }

    public Trigger rightYUpTrigger(double threshold) {
        return rightYUpTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightYUpTrigger() {
        return rightYUpTrigger(0.7);
    }
}
