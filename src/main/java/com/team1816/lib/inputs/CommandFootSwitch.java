package com.team1816.lib.inputs;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandFootSwitch {

    private final FootSwitch m_hid;

    public CommandFootSwitch(int port) {
        this(new FootSwitch(port));
    }

    public CommandFootSwitch(FootSwitch hid) {
        m_hid = hid;
    }

    public FootSwitch getHID() {
        return m_hid;
    }


    private Trigger hidButton(int buttonNumber, EventLoop loop) {
        if (loop == null) {
            return new Trigger(() -> m_hid.getRawButton(buttonNumber));
        } else {
            return new Trigger(loop, () -> m_hid.getRawButton(buttonNumber));
        }
    }



    public Trigger left() {
        return hidButton(FootSwitch.Pedal.kLeft.value, null);
    }
    public Trigger left(EventLoop loop) {
        return hidButton(FootSwitch.Pedal.kLeft.value, loop);
    }

    public Trigger center() {
        return hidButton(FootSwitch.Pedal.kCenter.value, null);
    }
    public Trigger center(EventLoop loop) {
        return hidButton(FootSwitch.Pedal.kCenter.value, loop);
    }

    public Trigger right() {
        return hidButton(FootSwitch.Pedal.kRight.value, null);
    }
    public Trigger right(EventLoop loop) {
        return hidButton(FootSwitch.Pedal.kRight.value, loop);
    }
}
