package com.team1816.season;

import com.team1816.lib.commands.GreenCommand;
import com.team1816.lib.subsystems.LedManager;

public class InTheZoneCommand extends GreenCommand {

    private final LedManager.RobotLEDStateEvent robotStateEvent = pubsub.GetEvent(LedManager.RobotLEDStateEvent.class);

    @Override
    public void initialize() {
        robotStateEvent.Publish(LedManager.LEDControlState.BLINK);
    }

    @Override
    public void end(boolean interrupted) {
        robotStateEvent.Publish(LedManager.LEDControlState.SOLID);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
