package com.team1816.season.tests;

import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Intake;
import com.team1816.season.subsystems.Shooter;
import org.junit.jupiter.api.BeforeEach;

public class ShooterTest {

    private Shooter shooter;

    @BeforeEach
    public void testInit() {
        shooter = Singleton.get(Shooter.class);
    }
}
