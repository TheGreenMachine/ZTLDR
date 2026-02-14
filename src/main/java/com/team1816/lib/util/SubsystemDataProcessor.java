package com.team1816.lib.util;

import java.util.List;

public class SubsystemDataProcessor implements Runnable {
    public static final int LOOP_TIME = 20;
    private double timestamp = 0.0;
    private final List<IDataRefresher> dataRefreshers;

    public SubsystemDataProcessor(IDataRefresher IODataRefresher) {
        dataRefreshers = List.of(IODataRefresher);
    }

    public static void createAndStartSubsystemDataProcessor(IDataRefresher dataRefresher) {
        new Thread(new SubsystemDataProcessor(dataRefresher)).start();
    }

    public void run() {
        while (true) {
            timestamp = System.currentTimeMillis();
            for (IDataRefresher IODataRefresher : dataRefreshers) {
                IODataRefresher.readFromHardware();
            }

            try {
                var difference = System.currentTimeMillis() - timestamp;
                if (difference < LOOP_TIME) {
                    Thread.sleep((long) (LOOP_TIME - difference));
                }
            } catch (InterruptedException e) {
            }
        }
    }

    public interface IDataRefresher {
        void readFromHardware();
    }
}
