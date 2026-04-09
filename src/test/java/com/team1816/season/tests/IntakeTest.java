package com.team1816.season.tests;

import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Intake;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class IntakeTest {

    private Intake intake;

    @BeforeEach
    public void testInit() {
        intake = Singleton.get(Intake.class);
    }

    @Test
    public void testReadFromHardware() {

    }

    @Test
    public void testPeriodic() {
        Intake.IntakeState intakeFooState = Intake.IntakeState.INTAKE;
        intake.setWantedState(intakeFooState); //<- Foo value
        intake.periodic();

        assertEquals(.4, intake.getState().getIntakeMotorValue(), 0, "Intake Motor is screwy");
//        assertEquals(40, intake.getState().getFlipperMotorPosition(), 100, "Flipper Motor is screwy"); //<- Will figure out later (lazy)
    }

    @AfterEach
    public void clearTest() {
        intake = null; //<- Is this okay
    }

}
