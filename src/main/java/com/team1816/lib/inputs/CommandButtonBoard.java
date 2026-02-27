package com.team1816.lib.inputs;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@SuppressWarnings("MethodName")
public class CommandButtonBoard {

    private final ButtonBoard m_hid;

    public CommandButtonBoard(int port) {
        this(new ButtonBoard(port));
    }

    public CommandButtonBoard(ButtonBoard hid) {
        m_hid = hid;
    }

    public ButtonBoard getHID() {
        return m_hid;
    }


    private Trigger hidButton(int buttonNumber, EventLoop loop) {
        if (loop == null) {
            return new Trigger(() -> m_hid.getRawButton(buttonNumber));
        } else {
            return new Trigger(loop, () -> m_hid.getRawButton(buttonNumber));
        }
    }



    public Trigger topLeft() {
        return hidButton(ButtonBoard.Button.kTopLeft.value, null);
    }
    public Trigger topLeft(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kTopLeft.value, loop);
    }

    public Trigger topCenter() {
        return hidButton(ButtonBoard.Button.kTopCenter.value, null);
    }
    public Trigger topCenter(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kTopCenter.value, loop);
    }

    public Trigger topRight() {
        return hidButton(ButtonBoard.Button.kTopRight.value, null);
    }
    public Trigger topRight(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kTopRight.value, loop);
    }



    public Trigger middleLeft() {
        return hidButton(ButtonBoard.Button.kMiddleLeft.value, null);
    }
    public Trigger middleLeft(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kMiddleLeft.value, loop);
    }

    public Trigger middleCenter() {
        return hidButton(ButtonBoard.Button.kMiddleCenter.value, null);
    }
    public Trigger middleCenter(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kMiddleCenter.value, loop);
    }

    public Trigger middleRight() {
        return hidButton(ButtonBoard.Button.kMiddleRight.value, null);
    }
    public Trigger middleRight(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kMiddleRight.value, loop);
    }



    public Trigger bottomLeft() {
        return hidButton(ButtonBoard.Button.kBottomLeft.value, null);
    }
    public Trigger bottomLeft(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kBottomLeft.value, loop);
    }

    public Trigger bottomCenter() {
        return hidButton(ButtonBoard.Button.kBottomCenter.value, null);
    }
    public Trigger bottomCenter(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kBottomCenter.value, loop);
    }

    public Trigger bottomRight() {
        return hidButton(ButtonBoard.Button.kBottomRight.value, null);
    }
    public Trigger bottomRight(EventLoop loop) {
        return hidButton(ButtonBoard.Button.kBottomRight.value, loop);
    }
}
