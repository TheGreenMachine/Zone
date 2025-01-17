package com.team1816.lib.hardware;

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

        return defaultConfig;
    }
}
