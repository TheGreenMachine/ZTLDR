package com.team1816.season.tests;

import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.math.geometry.Translation2d;
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
        //todo: Seriously fix this
//        boolean rightFooSensorTriggered;
//        boolean leftFooSensorTriggered;
//        boolean fooSensorValueHaveBeenSet;

//        shooter.setLeftSensorState(true);
//        shooter.setRightSensorState(true);
//        shooter.setSensorValuesHaveBeenSet(true);

//        rightFooSensorTriggered = shooter.getRightSensorState(); //This might be redundant
//        leftFooSensorTriggered = shooter.getLeftSensorState();
//        fooSensorValueHaveBeenSet = shooter.getSensorValuesHaveBeenSet();

//        shooter.readFromHardware();
//
//        assertFalse(shooter.getLeftPreviousState(), "Previous left sensor are not being set right in readFromHardware");
//        assertFalse(shooter.getRightPreviousState(), "Previous right sensors are not being set right in readFromHardware");
//
//        assertTrue(shooter.getLeftSensorState(), "Left sensor are not being set right in readFromHardware");
//        assertTrue(shooter.getRightSensorState(), "Right sensors are not being set right in readFromHardware");

    }

    @Test
    public void testPeriodic() {
        //todo: Seriously fix this
//        DriverStation.isEnabled(); //<- Is probably not how we want this to work ╮(ᵕ—ᴗ—)╭
//        Translation2d fooTarget = Translation2d.kZero;
//
//        Shooter.ShooterDistanceState fooState = Shooter.ShooterDistanceState.PRESET_CLOSE;
//
//        shooter.setWantedDistanceState(fooState);
//
//        shooter.periodic();
//
//        assertEquals(0.064, shooter.getWantedShooterDistanceState().getInclineAngleDegrees(), 100, "Preset Angles are screwy");
//        assertEquals(35, shooter.getWantedShooterDistanceState().getLaunchVelocityRPS(), 100, "Preset Velocities are screwy");
//        //Do we need each preset state to be tested or is one good?
//
//        fooState = Shooter.ShooterDistanceState.AUTOMATIC;
//
//        shooter.setWantedDistanceState(fooState);
//        /*todo: Have some way to check that based on the robot pose (not sure how to make foo one with how it currently set up), that it choose the right targe
//           Also, testing aimInclineAndLaunchersAtTarget to make sure the autoAim is pointing at *roughly* the right direction*/
//
//        //Do we need the calibration method test (probably not...)
//        //Question to Henry: Why did you change the enum distance names, but not the yaml ones?? <- Doesn't need a serious answer
    }

    @AfterEach
    public void clearTest() {
            shooter = null; //<- Is this okay??
    }
}
