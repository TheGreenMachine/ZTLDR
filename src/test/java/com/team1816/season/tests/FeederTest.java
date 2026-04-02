package com.team1816.season.tests;

import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Feeder;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class FeederTest {

    private Feeder feeder;

    @BeforeEach
    public void testInit() {
        feeder = Singleton.get(Feeder.class);
    }

    @Test
    public void testPeriodic() {
        Feeder.FeederState feederFooState = Feeder.FeederState.FEEDING;
        feeder.setWantedState(feederFooState); //<- Foo value
        feeder.periodic();

        assertEquals(.7, feeder.getState().getFeedMotorDutyCycle(), 0, "Feed Motor is screwy");
    }

    @AfterEach
    public void clearTest() {
        feeder = null; //<- Is this okay
    }
}
