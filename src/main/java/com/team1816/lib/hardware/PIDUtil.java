package com.team1816.lib.hardware;

import com.ctre.phoenix6.signals.GravityTypeValue;

public final class PIDUtil {
    private PIDUtil() {
    }

    /**
     * Creates a {@link PIDSlotConfiguration} with all values at 0.
     */
    public static PIDSlotConfiguration createDefaultPIDSlotConfig() {
        PIDSlotConfiguration defaultConfig = new PIDSlotConfiguration();

        defaultConfig.kP = 0.0;
        defaultConfig.kI = 0.0;
        defaultConfig.kD = 0.0;
        defaultConfig.kV = 0.0;
        defaultConfig.kS = 0.0;
        defaultConfig.kA = 0.0;
        defaultConfig.kG = 0.0;
        defaultConfig.gravityType = GravityTypeValue.Elevator_Static.name(); // default gravity type

        return defaultConfig;
    }
}
