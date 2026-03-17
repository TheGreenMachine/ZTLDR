package com.team1816.lib.inputs;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class FootSwitch extends GenericHID implements Sendable {
    /**
     * Represents a pedal on a Foot Switch FS3-P.
     */
    public enum Pedal { // TODO: Find what these values need to be.
        /**
         * Left pedal.
         */
        kLeft(0),
        /**
         * Center pedal.
         */
        kCenter(1),
        /**
         * Right pedal.
         */
        kRight(2);

        /**
         * Pedal value.
         */
        public final int value;

        Pedal(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            // Remove leading `k`
            return this.name().substring(1) + "Button";
        }
    }

    public FootSwitch(final int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_Joystick, port + 1);
    }


    public boolean getLeftPedal() {
        return getRawButton(Pedal.kLeft.value);
    }
    public boolean getLeftPedalPressed() {
        return getRawButtonPressed(Pedal.kLeft.value);
    }
    public boolean getLeftPedalReleased() {
        return getRawButtonReleased(Pedal.kLeft.value);
    }
    public BooleanEvent left(EventLoop loop) {
        return button(Pedal.kLeft.value, loop);
    }


    public boolean getCenterPedal() {
        return getRawButton(Pedal.kCenter.value);
    }
    public boolean getCenterPedalPressed() {
        return getRawButtonPressed(Pedal.kCenter.value);
    }
    public boolean getCenterPedalReleased() {
        return getRawButtonReleased(Pedal.kCenter.value);
    }
    public BooleanEvent center(EventLoop loop) {
        return button(Pedal.kCenter.value, loop);
    }


    public boolean getRightPedal() {
        return getRawButton(Pedal.kRight.value);
    }
    public boolean getRightPedalPressed() {
        return getRawButtonPressed(Pedal.kRight.value);
    }
    public boolean getRightPedalReleased() {
        return getRawButtonReleased(Pedal.kRight.value);
    }
    public BooleanEvent right(EventLoop loop) {
        return button(Pedal.kRight.value, loop);
    }

    //Lets dashboard get buttons but not set them
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.publishConstString("ControllerType", "FootSwitch");
        builder.addBooleanProperty("Left",      this::getLeftPedal,      null);
        builder.addBooleanProperty("Center",    this::getCenterPedal,    null);
        builder.addBooleanProperty("Right",     this::getRightPedal,     null);
    }
}
