package com.team1816.lib.util;

import com.team1816.lib.Singleton;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ShotLookupTest {

    @Test
    void GetRps0() {
        double[] firstlist = {2,3};
        String[] secondlist = {"2x^3+x^2+x+3","+20x^5+5x^2+2"};
        ShotLookup shotLookup = new ShotLookup(firstlist,secondlist);
        assertEquals(62627, shotLookup.getRPS(1, 5));
    }

}
