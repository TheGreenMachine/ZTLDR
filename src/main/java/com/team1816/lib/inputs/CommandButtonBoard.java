// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1816.lib.inputs;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;


@SuppressWarnings("MethodName")
public class CommandButtonBoard extends CommandGenericHID {
    private final ButtonBoard m_hid;

    public CommandButtonBoard(int port) {
        super(port);
        m_hid = new ButtonBoard(port);
    }

    @Override
    public ButtonBoard getHID() {
        return m_hid;
    }


    //topLeft
    public Trigger topLeft() {
        return topLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger topLeft(EventLoop loop) {
        return button(ButtonBoard.Button.kTopLeft.value, loop);
    }


    //topCenter
    public Trigger topCenter() {
        return topCenter(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger topCenter(EventLoop loop) {
        return button(ButtonBoard.Button.kTopCenter.value, loop);
    }


    //topRight
    public Trigger topRight() {
        return topRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger topRight(EventLoop loop) {
        return button(ButtonBoard.Button.kTopRight.value, loop);
    }


    //middleLeft
    public Trigger middleLeft() {
        return middleLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger middleLeft(EventLoop loop) {
        return button(ButtonBoard.Button.kMiddleLeft.value, loop);
    }


    //middleCenter
    public Trigger middleCenter() {
        return middleCenter(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger middleCenter(EventLoop loop) {
        return button(ButtonBoard.Button.kMiddleCenter.value, loop);
    }


    //middleRight
    public Trigger middleRight() {
        return middleRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger middleRight(EventLoop loop) {
        return button(ButtonBoard.Button.kMiddleRight.value, loop);
    }


    //bottomLeft
    public Trigger bottomLeft() {
        return bottomLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger bottomLeft(EventLoop loop) {
        return button(ButtonBoard.Button.kBottomLeft.value, loop);
    }

    //bottomCenter
    public Trigger bottomCenter() {
        return bottomCenter(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger bottomCenter(EventLoop loop) {
        return button(ButtonBoard.Button.kBottomCenter.value, loop);
    }


    //bottomRight
    public Trigger bottomRight() {
        return bottomRight(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger bottomRight(EventLoop loop) {
        return button(ButtonBoard.Button.kBottomRight.value, loop);
    }
}
