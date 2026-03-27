package com.team1816.lib.util;

import com.team1816.lib.Singleton;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ShotLookupTest {

    @Test
    void GetRps0() {
        double[] firstlist = {2,3};
        String[] secondlist = {"2x+3","5x+2"};
        ShotLookup shotLookup = new ShotLookup(firstlist,secondlist);
        assertEquals(17, shotLookup.getRPS(1, 3.0));
    }

}
