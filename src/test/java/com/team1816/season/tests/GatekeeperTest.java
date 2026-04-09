package com.team1816.season.tests;

import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Gatekeeper;
import com.team1816.season.subsystems.Superstructure;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class GatekeeperTest {

    private Gatekeeper gatekeeper;

    @BeforeEach
    public void testInit() {
        gatekeeper = Singleton.get(Gatekeeper.class);
    }

    @Test
    public void testPeriodic() {
        Gatekeeper.GatekeeperState gatekeeperFooState = Gatekeeper.GatekeeperState.OPEN;
        gatekeeper.setWantedState(gatekeeperFooState); //<- Foo value
        gatekeeper.periodic();

        assertEquals(60.0, gatekeeper.getState().getTopMotorValue(), 0, "Top Motor is screwy");
//        assertEquals(40, gatekeeper.getState().getBottomMotorValue(), 100, "Bottom Motor is screwy");
    }

    @AfterEach
    public void clearTest() {
        gatekeeper = null; //<- Is this okay
    }
}
