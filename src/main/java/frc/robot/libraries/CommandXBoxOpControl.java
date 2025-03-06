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

    public static final double kThreshold = 0.7;

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
        return rightYUpTrigger(kThreshold);
    }

    public Trigger rightYDownTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(XboxController.Axis.kRightY.value, threshold, loop);
    }

    public Trigger rightYDownTrigger(double threshold) {
        return rightYDownTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightYDownTrigger() {
        return rightYDownTrigger(kThreshold);
    }

    public Trigger rightXLeftTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(XboxController.Axis.kRightX.value, threshold, loop);
    }

    public Trigger rightXLeftTrigger(double threshold) {
        return rightXLeftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightXLeftTrigger() {
        return rightXLeftTrigger(kThreshold);
    }
    
    public Trigger rightXRightTrigger(double threshold, EventLoop loop) {
        return axisLessThan(XboxController.Axis.kRightX.value, threshold, loop);
    }

    public Trigger rightXRightTrigger(double threshold) {
        return rightXRightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightXRightTrigger() {
        return rightXRightTrigger(kThreshold);
    }

    public Trigger leftYUpTrigger(double threshold, EventLoop loop) {
        return axisLessThan(XboxController.Axis.kLeftY.value, threshold, loop);
    }

    public Trigger leftYUpTrigger(double threshold) {
        return leftYUpTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftYUpTrigger() {
        return leftYUpTrigger(kThreshold);
    }

    public Trigger leftYDownTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(XboxController.Axis.kLeftY.value, threshold, loop);
    }

    public Trigger leftYDownTrigger(double threshold) {
        return leftYDownTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftYDownTrigger() {
        return leftYDownTrigger(kThreshold);
    }

    public Trigger leftXLeftTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(XboxController.Axis.kLeftX.value, threshold, loop);
    }

    public Trigger leftXLeftTrigger(double threshold) {
        return leftXLeftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftXLeftTrigger() {
        return leftXLeftTrigger(kThreshold);
    }
    
    public Trigger leftXRightTrigger(double threshold, EventLoop loop) {
        return axisLessThan(XboxController.Axis.kLeftX.value, threshold, loop);
    }

    public Trigger leftXRightTrigger(double threshold) {
        return leftXRightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftXRightTrigger() {
        return leftXRightTrigger(kThreshold);
    }
}
