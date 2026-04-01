package com.team1816.season.tests;

import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class ShooterTest {
    private Shooter shooter;

    @BeforeEach
    public void testInit() {
            shooter = Singleton.get(Shooter.class);
        }

    @Test
    public void testReadFromHardware() {
//        Shooter.ShooterDistanceState shooterFooDistanceState = Shooter.ShooterDistanceState.AUTOMATIC;
//        shooter.setWantedDistanceState(shooterFooDistanceState);
        boolean rightFooSensorTriggered;
        boolean leftFooSensorTriggered;
        boolean fooSensorValueHaveBeenSet;

        shooter.setLeftSensorState(true);
        shooter.setRightSensorState(true);
        shooter.setSensorValuesHaveBeenSet(true);

        rightFooSensorTriggered = shooter.getRightSensorState(); //This might be redundant
        leftFooSensorTriggered = shooter.getLeftSensorState();
        fooSensorValueHaveBeenSet = shooter.getSensorValuesHaveBeenSet();

        shooter.readFromHardware();

        assertTrue(shooter.getLeftPreviousState(), "Previous left sensor are not being set right in readFromHardware, fix it meow >:3");
        assertTrue(shooter.getRightPreviousState(), "Previous right sensors are not being set right in readFromHardware, fix it meow >:3");

        assertFalse(shooter.getLeftSensorState(), "Left sensor are not being set right in readFromHardware, fix it meow >:3");
        assertFalse(shooter.getRightSensorState(), "Right sensors are not being set right in readFromHardware, fix it meow >:3");

    }

    @Test
    public void testPeriodic() {
        DriverStation.isEnabled(); //<- Is probably not how we want this to work ╮(ᵕ—ᴗ—)╭

        shooter.periodic();


    }

    @AfterEach
    public void clearTest() {
            shooter = null; //<- Is this going to work
    }
}
