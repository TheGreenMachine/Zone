package com.team1816.lib.subsystems;

import com.team1816.lib.hardware.PIDSlotConfiguration;

/**
 * Base PID interface for library subsystems and provides a fully functional juncture to configure PID
 */
public interface PidProvider {
    PIDSlotConfiguration getPIDConfig();

    default String pidToString() {
        return String.format(
                "kP = %f, kI = %f, kD = %f, kV = %f, kS = %f, kA = %f kG = %f",
                getPIDConfig().kP,
                getPIDConfig().kI,
                getPIDConfig().kD,
                getPIDConfig().kV,
                getPIDConfig().kS,
                getPIDConfig().kA,
                getPIDConfig().kG
        );
    }
}
