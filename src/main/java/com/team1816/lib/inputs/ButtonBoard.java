package com.team1816.lib.inputs;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;


public class ButtonBoard extends GenericHID implements Sendable {
    // These values may be wrong
    public enum Button {
        // Top-left button
        kTopLeft(2),
        // Top-center button
        kTopCenter(5),
        // Top-right button
        kTopRight(12),
        // Middle-left button
        kMiddleLeft(3),
        // Middle-center button
        kMiddleCenter(4),
        // Middle-right button
        kMiddleRight(10),
        // Bottom-left button
        kBottomLeft(9),
        // Bottom-center button
        kBottomCenter(11),
        // Bottom-right button
        kBottomRight(13);

        // Button value.
        public final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            // Remove leading `k`
            return this.name().substring(1) + "Button";
        }
    }

    public ButtonBoard(final int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_Joystick, port + 1);
    }


    public boolean getTopLeftButton() {
        return getRawButton(Button.kTopLeft.value);
    }
    public boolean getTopLeftButtonPressed() {
        return getRawButtonPressed(Button.kTopLeft.value);
    }
    public boolean getTopLeftButtonReleased() {
        return getRawButtonReleased(Button.kTopLeft.value);
    }
    public BooleanEvent topLeft(EventLoop loop) {
        return button(Button.kTopLeft.value, loop);
    }


    public boolean getTopCenterButton() {
        return getRawButton(Button.kTopCenter.value);
    }
    public boolean getTopCenterButtonPressed() {
        return getRawButtonPressed(Button.kTopCenter.value);
    }
    public boolean getTopCenterButtonReleased() {
        return getRawButtonReleased(Button.kTopCenter.value);
    }
    public BooleanEvent topCenter(EventLoop loop) {
        return button(Button.kTopCenter.value, loop);
    }


    public boolean getTopRightButton() {
        return getRawButton(Button.kTopRight.value);
    }
    public boolean getTopRightButtonPressed() {
        return getRawButtonPressed(Button.kTopRight.value);
    }
    public boolean getTopRightButtonReleased() {
        return getRawButtonReleased(Button.kTopRight.value);
    }
    public BooleanEvent topRight(EventLoop loop) {
        return button(Button.kTopRight.value, loop);
    }


    public boolean getMiddleLeftButton() {
        return getRawButton(Button.kMiddleLeft.value);
    }
    public boolean getMiddleLeftButtonPressed() {
        return getRawButtonPressed(Button.kMiddleLeft.value);
    }
    public boolean getMiddleLeftButtonReleased() {
        return getRawButtonReleased(Button.kMiddleLeft.value);
    }
    public BooleanEvent middleLeft(EventLoop loop) {
        return button(Button.kMiddleLeft.value, loop);
    }


    public boolean getMiddleCenterButton() {
        return getRawButton(Button.kMiddleCenter.value);
    }
    public boolean getMiddleCenterButtonPressed() {
        return getRawButtonPressed(Button.kMiddleCenter.value);
    }
    public boolean getMiddleCenterButtonReleased() {
        return getRawButtonReleased(Button.kMiddleCenter.value);
    }
    public BooleanEvent middleCenter(EventLoop loop) {
        return button(Button.kMiddleCenter.value, loop);
    }


    public boolean getMiddleRightButton() {
        return getRawButton(Button.kMiddleRight.value);
    }
    public boolean getMiddleRightButtonPressed() {
        return getRawButtonPressed(Button.kMiddleRight.value);
    }
    public boolean getMiddleRightButtonReleased() {
        return getRawButtonReleased(Button.kMiddleRight.value);
    }
    public BooleanEvent middleRight(EventLoop loop) {
        return button(Button.kMiddleRight.value, loop);
    }


    public boolean getBottomLeftButton() {
        return getRawButton(Button.kBottomLeft.value);
    }
    public boolean getBottomLeftButtonPressed() {
        return getRawButtonPressed(Button.kBottomLeft.value);
    }
    public boolean getBottomLeftButtonReleased() {
        return getRawButtonReleased(Button.kBottomLeft.value);
    }
    public BooleanEvent bottomLeft(EventLoop loop) {
        return button(Button.kBottomLeft.value, loop);
    }


    public boolean getBottomCenterButton() {
        return getRawButton(Button.kBottomCenter.value);
    }
    public boolean getBottomCenterButtonPressed() {
        return getRawButtonPressed(Button.kBottomCenter.value);
    }
    public boolean getBottomCenterButtonReleased() {
        return getRawButtonReleased(Button.kBottomCenter.value);
    }
    public BooleanEvent bottomCenter(EventLoop loop) {
        return button(Button.kBottomCenter.value, loop);
    }


    public boolean getBottomRightButton() {
        return getRawButton(Button.kBottomRight.value);
    }
    public boolean getBottomRightButtonPressed() {
        return getRawButtonPressed(Button.kBottomRight.value);
    }
    public boolean getBottomRightButtonReleased() {
        return getRawButtonReleased(Button.kBottomRight.value);
    }
    public BooleanEvent bottomRight(EventLoop loop) {
        return button(Button.kBottomRight.value, loop);
    }

    //Lets dashboard get buttons but not set them
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.publishConstString("ControllerType", "ButtonBoard");
        builder.addBooleanProperty("TopLeft",      this::getTopLeftButton,      null);
        builder.addBooleanProperty("TopCenter",    this::getTopCenterButton,    null);
        builder.addBooleanProperty("TopRight",     this::getTopRightButton,     null);
        builder.addBooleanProperty("MiddleLeft",   this::getMiddleLeftButton,   null);
        builder.addBooleanProperty("MiddleCenter", this::getMiddleCenterButton, null);
        builder.addBooleanProperty("MiddleRight",  this::getMiddleRightButton,  null);
        builder.addBooleanProperty("BottomLeft",   this::getBottomLeftButton,   null);
        builder.addBooleanProperty("BottomCenter", this::getBottomCenterButton, null);
        builder.addBooleanProperty("BottomRight",  this::getBottomRightButton,  null);
    }
}
