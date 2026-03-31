package com.team1816.season.tests;

import com.team1816.lib.Singleton;
import com.team1816.season.subsystems.Intake;
import com.team1816.season.subsystems.Superstructure;
import org.junit.jupiter.api.BeforeEach;

public class SuperstructureTest {
    //Todo: To Ask, "do we NEED this?


    private Superstructure superstructure;

    @BeforeEach
    public void testInit() {
        superstructure = Singleton.get(Superstructure.class);
    }
}
